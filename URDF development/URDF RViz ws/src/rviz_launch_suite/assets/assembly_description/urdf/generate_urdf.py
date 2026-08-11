#!/usr/bin/env python3
"""Expand assembly.xacro into a standalone assembly.urdf.

The previous version of this script did the expansion with regexes, which
silently mangled anything it did not anticipate. This one hands the file to
the real xacro parser, so what lands in assembly.urdf is exactly what
robot_state_publisher would have seen.

    python3 generate_urdf.py

Mesh references are rewritten from ``file://$(find assembly_description)/...``
to ``package://assembly_description/...`` so the output works in RViz/Gazebo
without xacro. Pass --relative to emit ``../meshes/...`` paths instead, for
viewers that load the file straight off disk.
"""

import argparse
import os
import sys

import xacro

PKG = 'assembly_description'
FIND = f'file://$(find {PKG})/'
TOKEN = 'XACRO_PKG_ROOT/'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input', default='assembly.xacro')
    ap.add_argument('--output', default='assembly.urdf')
    ap.add_argument('--relative', action='store_true',
                    help='emit ../meshes/... instead of package:// paths')
    ap.add_argument('--standalone', action='store_true',
                    help='viewer-safe output: relative mesh paths, and strip the '
                         '<gazebo> and <transmission> blocks. Use this for '
                         'drag-and-drop URDF viewers, which cannot resolve '
                         'package:// and often crash on the SDF loop-closure '
                         'joints nested inside <gazebo>.')
    args = ap.parse_args()
    if args.standalone:
        args.relative = True

    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.join(here, args.input)
    dst = os.path.join(here, args.output)

    # $(find ...) needs a sourced ROS workspace to resolve. Stub it out so the
    # script also runs standalone, then rewrite the paths afterwards. Meshes
    # get a token we rewrite later; includes become plain relative paths,
    # which xacro resolves against the including file's directory.
    with open(src, encoding='utf-8') as f:
        text = f.read()
    text = text.replace(FIND + 'meshes/', TOKEN + 'meshes/')
    text = text.replace(f'$(find {PKG})/urdf/', '')
    stubbed = os.path.join(here, '.assembly.xacro.tmp')
    with open(stubbed, 'w', encoding='utf-8') as f:
        f.write(text)

    try:
        doc = xacro.process_file(stubbed)
        if args.standalone:
            robot = doc.documentElement
            for tag in ('gazebo', 'transmission'):
                for node in list(robot.getElementsByTagName(tag)):
                    node.parentNode.removeChild(node)
                    node.unlink()
        urdf = doc.toprettyxml(indent='  ')
    finally:
        os.remove(stubbed)

    replacement = '../meshes/' if args.relative else f'package://{PKG}/meshes/'
    urdf = urdf.replace(TOKEN + 'meshes/', replacement)

    if TOKEN in urdf:
        print('error: unresolved mesh path left in output', file=sys.stderr)
        return 1

    # collapse the blank lines toprettyxml leaves behind
    urdf = '\n'.join(line for line in urdf.splitlines() if line.strip())

    with open(dst, 'w', encoding='utf-8') as f:
        f.write(urdf + '\n')

    print(f'wrote {dst}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
