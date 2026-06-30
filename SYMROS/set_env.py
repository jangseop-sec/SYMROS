import os
import sys

KEYWORD = "ros2-nodds dependency"


def already_exists(lines):
  return any(KEYWORD in line for line in lines)


def add_ros_nodds_dependency(path, mode):
  
  if mode == "conc":
    TARGET_BLOCK = [
      "# ros2-nodds dependency\n",
      "add_compile_options(-w)\n",
      "set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)\n",
      "include_directories(${ros_nodds}/include)\n",
      "link_directories(${ros_nodds}/build-conc)\n",
      "link_libraries(ros2_no_dds)\n",
      "# ros2-nodds dependency\n"
    ]
  else:
    TARGET_BLOCK = [
      "# ros2-nodds dependency\n",
      "add_compile_options(-w)\n",
      "set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)\n",
      "include_directories(${ros_nodds}/include)\n",
      "link_directories(${ros_nodds}/build-sym)\n",
      "link_libraries(ros2_no_dds)\n",
      "include_directories(/home/ubuntu/symros/SYMROS/symros-libcxx/include)\n",
      "# ros2-nodds dependency\n"
    ]
  
  
  with open(path, "r", encoding="utf-8") as f:
    lines = f.readlines()

  if already_exists(lines):
    return

  new_lines = []
  inserted = False

  in_project = False
  paren_depth = 0

  for line in lines:
    new_lines.append(line)

    if inserted:
      continue

    stripped = line.strip()

    if not in_project and stripped.startswith("project("):
      in_project = True
      paren_depth += line.count("(") - line.count(")")

      if paren_depth <= 0:
        new_lines.append("\n")
        new_lines.extend(TARGET_BLOCK)
        inserted = True
        in_project = False

      continue

    if in_project:
      paren_depth += line.count("(") - line.count(")")

      if paren_depth <= 0:
        new_lines.append("\n")
        new_lines.extend(TARGET_BLOCK)
        inserted = True
        in_project = False
  if not inserted:
    new_lines.append("\n")
    new_lines.extend(TARGET_BLOCK)

  with open(path, "w", encoding="utf-8") as f:
    f.writelines(new_lines)

  print(f"[INSERTED] {path}")

def remove_ros_nodds_dependency(path, mode):
  if mode == "conc":
    TARGET_BLOCK = [
      "# ros2-nodds dependency\n",
      "add_compile_options(-w)\n",
      "set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)\n",
      "include_directories(${ros_nodds}/include)\n",
      "link_directories(${ros_nodds}/build-conc)\n",
      "link_libraries(ros2_no_dds)\n",
      "# ros2-nodds dependency\n"
    ]
  else:
    TARGET_BLOCK = [
      "# ros2-nodds dependency\n",
      "add_compile_options(-w)\n",
      "set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)\n",
      "include_directories(${ros_nodds}/include)\n",
      "link_directories(${ros_nodds}/build-sym)\n",
      "link_libraries(ros2_no_dds)\n",
      "include_directories(/home/ubuntu/symros/SYMROS/symros-libcxx/include)\n",
      "# ros2-nodds dependency\n"
    ]
  
  with open(path, "r", encoding="utf-8") as f:
    lines = f.readlines()

  new_lines = []
  i = 0
  changed = False

  while i < len(lines):
    if lines[i].strip() == TARGET_BLOCK[0][:-1]:
      match = True
      for j in range(len(TARGET_BLOCK)):
        if i + j >= len(lines) or lines[i + j].strip() != TARGET_BLOCK[j][:-1].strip():
          match = False
          break

      if match:
        i += len(TARGET_BLOCK)
        changed = True
        continue

    new_lines.append(lines[i])
    i += 1

  if changed:
    with open(path, "w", encoding="utf-8") as f:
      f.writelines(new_lines)
    print(f"[MODIFIED] {path}")

TARGET_INCLUDE_STRING = '#include "symros_string.hpp"'
TARGET_INCLUDE_MATH = '#include "symros_math.hpp"'

def add_include_symros_string(file_path, force_add=False):
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    has_include = any(TARGET_INCLUDE_STRING in line for line in lines)

    if has_include and not force_add:
        return

    lines.insert(0, TARGET_INCLUDE_STRING + "\n")

    with open(file_path, "w", encoding="utf-8") as f:
        f.writelines(lines)

    print(f"[ADD] {file_path}")

def remove_include_symros_string(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    new_lines = [line for line in lines if TARGET_INCLUDE_STRING not in line]

    if len(new_lines) != len(lines):
        with open(file_path, "w", encoding="utf-8") as f:
            f.writelines(new_lines)
        print(f"[REMOVE] {file_path}")

def add_include_symros_math(file_path, force_add=False):
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()
    if not "symros_" in content:
        print(f"[SKIP] {file_path} (no symros_math usage detected)")
        return
  
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    
    has_include = any(TARGET_INCLUDE_MATH in line for line in lines)

    if has_include and not force_add:
        return

    lines.insert(0, TARGET_INCLUDE_MATH + "\n")

    with open(file_path, "w", encoding="utf-8") as f:
        f.writelines(lines)

    print(f"[ADD] {file_path}")

def remove_include_symros_math(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    new_lines = [line for line in lines if TARGET_INCLUDE_MATH not in line]

    if len(new_lines) != len(lines):
        with open(file_path, "w", encoding="utf-8") as f:
            f.writelines(new_lines)
        print(f"[REMOVE] {file_path}")

def walk(root, mode, cmd):
  # CMakeSetting
  if mode == "symstring":
    for dirpath, _, filenames in os.walk(root):
      for file in filenames:
        if file.endswith((".cpp")):
          path = os.path.join(dirpath, file)
          if cmd == "add":
            add_include_symros_string(path)
          else:
            remove_include_symros_string(path)
    return

  if mode == "symmath":
    for dirpath, _, filenames in os.walk(root):
      for file in filenames:
        if file.endswith((".cpp")) or file.endswith(".hpp") or file.endswith((".c")) or file.endswith(".h"):
          path = os.path.join(dirpath, file)
          if cmd == "add":
            add_include_symros_math(path)
          else:
            remove_include_symros_math(path)
    return
    
  for dirpath, _, filenames in os.walk(root):
    for file in filenames:
      if file == "CMakeLists.txt":
        if cmd == "add":
          add_ros_nodds_dependency(os.path.join(dirpath, file), mode)
        else:
          remove_ros_nodds_dependency(os.path.join(dirpath, file), mode)
          

if __name__ == "__main__":
  folder = sys.argv[1]
  mode = sys.argv[2]
  cmd = sys.argv[3]
  walk(folder, mode, cmd)
