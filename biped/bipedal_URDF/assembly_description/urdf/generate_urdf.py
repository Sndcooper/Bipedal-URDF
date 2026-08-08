import os
import re

def read_inner_xml(filepath):
    with open(filepath, 'r') as f:
        content = f.read()
    # Remove xml declaration if present
    content = re.sub(r'<\?xml.*?\?>', '', content)
    # Extract everything inside <robot> ... </robot>
    match = re.search(r'<robot.*?>(.*)</robot>', content, re.DOTALL)
    if match:
        return match.group(1).strip()
    return ""

def main():
    with open('assembly.xacro', 'r') as f:
        xacro_content = f.read()

    # Replace includes
    includes = re.findall(r'<xacro:include filename="\$\(find assembly_description\)/urdf/(.*?)" />', xacro_content)
    
    for inc in includes:
        inc_content = read_inner_xml(inc)
        xacro_content = re.sub(rf'<xacro:include filename="\$\(find assembly_description\)/urdf/{inc}" />', inc_content, xacro_content)
        
    # Replace mesh paths
    # From: file://$(find assembly_description)/meshes/
    # To: package://assembly_description/meshes/
    xacro_content = xacro_content.replace('file://$(find assembly_description)/meshes/', 'package://assembly_description/meshes/')
    
    # Strip xacro namespace if desired, though not strictly necessary
    xacro_content = xacro_content.replace('xmlns:xacro="http://www.ros.org/wiki/xacro"', '')
    
    with open('assembly.urdf', 'w') as f:
        f.write(xacro_content)

if __name__ == '__main__':
    main()
