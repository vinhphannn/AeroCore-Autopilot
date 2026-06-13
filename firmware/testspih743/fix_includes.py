import os
import re

FC_CORE_DIR = os.path.abspath("FC_Core")

def find_file(filename):
    for root, dirs, files in os.walk(FC_CORE_DIR):
        if filename in files:
            return os.path.join(root, filename)
    return None

def fix_includes_in_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Find all includes that use quotes
    # #include "../something/file.h"
    lines = content.split('\n')
    modified = False
    for i, line in enumerate(lines):
        match = re.match(r'^#include\s+"([^"]+)"', line)
        if match:
            include_path = match.group(1)
            # if it's already just "filename.h" and it exists in same dir, skip
            # but if it has ../ or we can't find it
            basename = os.path.basename(include_path)
            actual_path = find_file(basename)
            if actual_path:
                # compute relative path from current file to actual_path
                rel_path = os.path.relpath(actual_path, os.path.dirname(filepath))
                if rel_path != include_path:
                    lines[i] = f'#include "{rel_path}"'
                    modified = True

    if modified:
        with open(filepath, 'w') as f:
            f.write('\n'.join(lines))

for root, dirs, files in os.walk(FC_CORE_DIR):
    for file in files:
        if file.endswith('.cpp') or file.endswith('.hpp') or file.endswith('.h') or file.endswith('.c'):
            fix_includes_in_file(os.path.join(root, file))

print("Includes fixed.")
