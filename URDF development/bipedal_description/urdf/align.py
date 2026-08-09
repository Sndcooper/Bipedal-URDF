import re
import math

def process_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    femurs = [
        ('right_lower_femur', 'right_lower_hip_joint', 'right_knee_joint'),
        ('left_lower_femur', 'left_lower_hip_joint', 'left_lower_knee_to_linkage2'),
        ('left_upper_femur', 'left_upper_hip_joint', 'left_knee_joint'),
        ('right_upper_femur', 'right_upper_hip_joint', 'right_upper_knee_to_linkage1')
    ]

    for link_name, parent_joint, child_joint in femurs:
        # 1. Find child joint origin
        child_pattern = rf'(<joint name="{child_joint}".*?<origin xyz=")([^"]+)(")'
        match = re.search(child_pattern, content, re.DOTALL)
        if not match:
            print(f"Child joint {child_joint} not found in {filepath}")
            continue
        
        xyz_str = match.group(2)
        x, y, z = map(float, xyz_str.split())
        
        # Calculate yaw and length
        yaw = math.atan2(y, x)
        length = math.hypot(x, y)
        
        # New xyz for child joint
        new_xyz = f"{length:.6f} 0.0 {z}"
        
        # Replace child joint origin
        content = content[:match.start(2)] + new_xyz + content[match.end(2):]
        
        # 2. Update parent joint rpy (we apply -yaw to parent so the X-axis points where it used to point)
        # Note: We must ensure we don't accidentally replace a different origin. 
        # The parent joint has <origin xyz="..." rpy="0 0 0"/>
        parent_pattern = rf'(<joint name="{parent_joint}".*?<origin xyz="[^"]+" rpy=")0 0 0(")'
        parent_match = re.search(parent_pattern, content, re.DOTALL)
        if parent_match:
            new_rpy = f"0 0 {-yaw:.5f}"
            content = content[:parent_match.start(2)] + new_rpy + content[parent_match.end(2):]
            
        # 3. Update link visual and collision origins (we apply +yaw to the mesh so it matches the old physical position)
        link_pattern = rf'(<link name="{link_name}">.*?</link>)'
        link_match = re.search(link_pattern, content, re.DOTALL)
        if link_match:
            link_block = link_match.group(1)
            # Replace rpy="0 0 0" with rpy="0 0 yaw" inside the link block
            new_link_block = link_block.replace('rpy="0 0 0"', f'rpy="0 0 {yaw:.5f}"')
            content = content[:link_match.start(1)] + new_link_block + content[link_match.end(1):]

    with open(filepath, 'w') as f:
        f.write(content)
    print(f"Successfully processed {filepath}")

if __name__ == '__main__':
    process_file('bipedal_visual.urdf')
    process_file('bipedal.xacro')
