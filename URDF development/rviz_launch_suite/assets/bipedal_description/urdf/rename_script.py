import os

urdf_dir = r"c:\Users\vilas\Documents\CIR\ROS 2 hum\biped\bipedal_description\urdf"
meshes_dir = r"c:\Users\vilas\Documents\CIR\ROS 2 hum\biped\bipedal_description\meshes"

replacements = {
    "combined_motor_tibia__1__1": "left_tibia",
    "combined_motor_tibia_1": "right_tibia",
    "Combined_femur__3__1": "left_lower_femur",
    "Combined_femur__2__1": "right_upper_femur",
    "Combined_femur__1__1": "left_upper_femur",
    "Combined_femur_1": "right_lower_femur",
    "tibia_2__1__1": "inner_linkage_2",
    "tibia_2_1": "inner_linkage_1",
    "Revolute 10": "left_tibia_to_linkage1",
    "Revolute 1": "right_lower_hip_joint",
    "Revolute 2": "left_lower_hip_joint",
    "Revolute 3": "left_upper_hip_joint",
    "Revolute 4": "right_upper_hip_joint",
    "Revolute 5": "left_knee_joint",
    "Revolute 6": "right_knee_joint",
    "Revolute 7": "right_upper_knee_to_linkage1",
    "Revolute 8": "left_lower_knee_to_linkage2",
    "Revolute 9": "right_tibia_to_linkage2",
}

# 1. Rename the STL files in meshes directory
print("Renaming STL files...")
for filename in os.listdir(meshes_dir):
    if filename.endswith(".stl"):
        new_filename = filename
        for old, new in replacements.items():
            if old in new_filename:
                new_filename = new_filename.replace(old, new)
                break # break to avoid multiple replacements on same filename if one matched
        
        if new_filename != filename:
            old_path = os.path.join(meshes_dir, filename)
            new_path = os.path.join(meshes_dir, new_filename)
            os.rename(old_path, new_path)
            print(f"Renamed {filename} -> {new_filename}")

# 2. Update all URDF/XACRO/GAZEBO/TRANS files
print("\nUpdating text files...")
for filename in os.listdir(urdf_dir):
    filepath = os.path.join(urdf_dir, filename)
    if os.path.isfile(filepath) and not filename.endswith('.py'):
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
        
        new_content = content
        for old, new in replacements.items():
            new_content = new_content.replace(old, new)
            
        if new_content != content:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(new_content)
            print(f"Updated {filename}")

print("Done!")
