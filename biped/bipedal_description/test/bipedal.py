import xml.etree.ElementTree as ET
from collections import defaultdict
import copy

urdf_path = r"c:\Users\vilas\Documents\CIR\ROS 2 hum\biped\bipedal_export_temp_description\urdf\bipedal_export_temp_visual.urdf"
output_path = r"c:\Users\vilas\Documents\CIR\ROS 2 hum\biped\bipedal_export_temp_description\urdf\bipedal_clean_visual.urdf"

tree = ET.parse(urdf_path)
root = tree.getroot()

# First pass: collect all links, taking only the FIRST definition if duplicated
unique_links = {}
for child in root.findall('link'):
    name = child.attrib.get('name')
    if name not in unique_links:
        unique_links[name] = child

# Build joint graph
joints = root.findall('joint')
graph = defaultdict(list)
for j in joints:
    parent = j.find('parent').attrib.get('link')
    child = j.find('child').attrib.get('link')
    graph[parent].append((child, j))

visited_links = set()
visited_links.add('base_link')
tree_joints = []
queue = ['base_link']

# BFS to build the valid tree
while queue:
    current = queue.pop(0)
    for child, joint in graph[current]:
        if child not in visited_links:
            visited_links.add(child)
            tree_joints.append(joint)
            queue.append(child)

# Create a new root for the clean URDF
new_root = ET.Element("robot", name=root.attrib.get('name') + "_clean")

# Add only the visited links (first definition)
for link_name in visited_links:
    if link_name in unique_links:
        new_root.append(copy.deepcopy(unique_links[link_name]))

# Add only the joints that form the tree
for j in tree_joints:
    new_root.append(copy.deepcopy(j))

# Add materials if any
for mat in root.findall('material'):
    new_root.append(copy.deepcopy(mat))

# Write the new clean URDF
new_tree = ET.ElementTree(new_root)
ET.indent(new_tree, space="  ", level=0)
new_tree.write(output_path, encoding='utf-8', xml_declaration=True)

print(f"Clean URDF written to {output_path}")
print(f"Original links: {len(root.findall('link'))}, Unique visited links: {len(visited_links)}")
print(f"Original joints: {len(joints)}, Tree joints: {len(tree_joints)}")
