import os

# === 配置区 ===
folder_path = r"F:\数据集\color_images"   # 数据集文件夹路径
ext_list = [".jpg", ".png"]   # 只处理这些后缀（小写），如果不限制扩展名可设为 None
dry_run = False                # True = 仅预览，不实际删除；False = 真正删除

# === 脚本逻辑 ===
files = os.listdir(folder_path)
if ext_list:
    files = [f for f in files if os.path.splitext(f)[1].lower() in ext_list]
files.sort()  # 按文件名排序

delete_count = 0
for i, filename in enumerate(files, start=1):
    if i % 5 == 0:  # 每隔 10 个文件删除一个
        file_path = os.path.join(folder_path, filename)
        if dry_run:
            print(f"[预览] 将删除: {file_path}")
        else:
            os.remove(file_path)
            print(f"已删除: {file_path}")
        delete_count += 1

print(f"\n总计{'将删除' if dry_run else '已删除'} {delete_count} 个文件。")