import os

# === 配置区 ===
folder_path = r"F:\数据集\竖直放置"   # 数据集文件夹路径
prefix = "vertical_image"              # 新文件名前缀
start_num = 1                 # 起始编号
ext_list = [".jpg", ".png"]   # 只处理这些后缀（小写）

# === 脚本逻辑 ===
files = os.listdir(folder_path)
files = [f for f in files if os.path.splitext(f)[1].lower() in ext_list]
files.sort()  # 按文件名排序，确保顺序一致

num_digits = len(str(len(files)))  # 自动位数，例如 4 表示 0001

for i, filename in enumerate(files, start=start_num):
    old_path = os.path.join(folder_path, filename)
    ext = os.path.splitext(filename)[1]
    new_name = f"{prefix}_{str(i).zfill(num_digits)}{ext}"
    new_path = os.path.join(folder_path, new_name)
    os.rename(old_path, new_path)
    print(f"重命名: {filename} -> {new_name}")

print("批量改名完成！")