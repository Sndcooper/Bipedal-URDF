import re

files = ['bipedal_visual.urdf', 'bipedal.xacro']
for f in files:
    with open(f, 'r') as file:
        content = file.read()
    
    # Fix the messed up rpy replacements
    # The user/script replaced 'rpy="0 0 0"' with something like 'rpy="0 0 00 0 0.18740/>'
    
    # 1. Fix missing quotes
    content = re.sub(r'rpy="([0-9\.\- ]+)/>', r'rpy="\1"/>', content)
    
    # 2. Fix the weird '0 0 00 0 value'
    content = re.sub(r'rpy="0 0 00 0 (-?\d+\.\d+)"', r'rpy="0 0 \1"', content)
    content = re.sub(r'rpy="0 0 00 0 (-?\d+\.\d+)"/>', r'rpy="0 0 \1"/>', content)
    
    # Clean up any leftover double quotes if any
    content = content.replace('""', '"')

    with open(f, 'w') as file:
        file.write(content)
    print(f"Fixed XML in {f}")
