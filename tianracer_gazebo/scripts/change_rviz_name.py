#!/usr/bin/env python
import sys

# Check if the correct number of command-line arguments is provided
if len(sys.argv) != 2:
    print("Usage: python script.py <new_name>")
    sys.exit(1)

# Extract the new name from the command-line arguments
new_name = sys.argv[1]

# 读取原始文件
file_path = "/home/tianbot/tianbot_ws/src/tianracer/tianracer_gazebo/rviz/nav_tianracer_01.rviz"
with open(file_path, 'r', encoding='utf-8') as file:
    file_content = file.read()

# 替换字符串
new_content = file_content.replace("tianracer_01", new_name)

# 写入新文件
new_file_path = f"/home/tianbot/tianbot_ws/src/tianracer/tianracer_gazebo/rviz/nav_{new_name}.rviz"
with open(new_file_path, 'w', encoding='utf-8') as new_file:
    new_file.write(new_content)

print(f'文件已更新，新文件保存在: {new_file_path}')
