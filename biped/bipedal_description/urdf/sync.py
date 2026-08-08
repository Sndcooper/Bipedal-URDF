import re
with open('bipedal_visual.urdf', 'r') as f:
    urdf = f.read()
with open('bipedal.xacro', 'r') as f:
    xacro = f.read()

# Replace hip joints in xacro with those from urdf
for joint in ['right_lower_hip_joint', 'left_lower_hip_joint', 'left_upper_hip_joint', 'right_upper_hip_joint']:
    pattern = rf'<joint name="{joint}".*?</joint>'
    urdf_joint = re.search(pattern, urdf, re.DOTALL).group(0)
    xacro = re.sub(pattern, urdf_joint, xacro, flags=re.DOTALL)

with open('bipedal.xacro', 'w') as f:
    f.write(xacro)
