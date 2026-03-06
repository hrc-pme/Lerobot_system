import sys
import os

file_path = "/opt/ros/humble/local/lib/python3.10/dist-packages/rosbridge_library/internal/message_conversion.py"

if not os.path.exists(file_path):
    # Try alternate path
    file_path = "/opt/ros/humble/lib/python3.10/site-packages/rosbridge_library/internal/message_conversion.py"
    if not os.path.exists(file_path):
        print("File not found")
        sys.exit(1)

print(f"Patching {file_path}")

with open(file_path, "r") as f:
    content = f.read()

target = """def _from_object_inst(inst: ROSMessage, _rostype: str) -> dict:
    # Create an empty dict then populate with values from the inst
    msg = {}"""

replacement = """def _from_object_inst(inst: ROSMessage, _rostype: str) -> dict:
    if isinstance(inst, (bytes, bytearray)):
        return list(inst)

    # Create an empty dict then populate with values from the inst
    msg = {}"""

if target in content:
    new_content = content.replace(target, replacement)
    with open(file_path, "w") as f:
        f.write(new_content)
    print("Successfully patched.")
else:
    print("Target string not found.")
    # Debugging
    start_idx = content.find("def _from_object_inst")
    if start_idx != -1:
        print(f"Found definition at {start_idx}. content around there:")
        print(content[start_idx:start_idx+200])
    
