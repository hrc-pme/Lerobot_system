import sys
import os

file_path = "/opt/ros/humble/local/lib/python3.10/dist-packages/rosbridge_library/internal/message_conversion.py"

if not os.path.exists(file_path):
    # Try alternate path
    file_path = "/opt/ros/humble/lib/python3.10/site-packages/rosbridge_library/internal/message_conversion.py"
    if not os.path.exists(file_path):
        print(f"File not found at {file_path}")
        sys.exit(1)

print(f"Reading {file_path}")

with open(file_path, "r") as f:
    lines = f.readlines()
    # Print lines around 190
    start = max(0, 180)
    end = min(len(lines), 210)
    for i in range(start, end):
        print(f"{i+1}: {lines[i]}", end="")

    print("\n\nChecking for duplicate `_from_object_inst` definitions or where it is defined...")
    for i, line in enumerate(lines):
        if "_from_object_inst" in line and "def " in line:
            print(f"Definition at line {i+1}: {line}")
