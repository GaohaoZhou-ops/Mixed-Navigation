import open3d as o3d
import numpy as np
import yaml
import json # 导入json模块
from PIL import Image
import os
import argparse

# ==============================================================================
#  JSON 配置管理函数
# ==============================================================================

def manage_z_config(config_path, current_floor, min_z, max_z, floors_parent_dir):
    """
    管理z_config.json文件：更新当前楼层信息并清理无效条目。
    
    Args:
        config_path (str): z_config.json文件的完整路径。
        current_floor (str): 当前正在处理的楼层名称 (e.g., "floor1")。
        min_z (float): 使用的最小z值。
        max_z (float): 使用的最大z值。
        floors_parent_dir (str): 'floors' 目录的路径 (e.g., "resources/floors")。
    """
    print("\n--- 开始管理 z_config.json ---")
    
    # 1. 加载现有的JSON配置，如果文件不存在则创建一个空字典
    try:
        with open(config_path, 'r') as f:
            config_data = json.load(f)
        print(f"成功加载配置文件: {config_path}")
    except (FileNotFoundError, json.JSONDecodeError):
        config_data = {}
        print(f"配置文件不存在或为空，将创建一个新的。")

    # 2. 更新或添加当前楼层的Z值信息
    config_data[current_floor] = {"z_min": min_z, "z_max": max_z}
    print(f"已更新/添加 '{current_floor}' 的配置: z_min={min_z}, z_max={max_z}")

    # 3. 清理无效条目：删除JSON中存在但实际目录已不存在的楼层
    if not os.path.exists(floors_parent_dir):
        print(f"警告: 楼层父目录 '{floors_parent_dir}' 不存在，跳过清理步骤。")
    else:
        # 获取所有实际存在的楼层文件夹名称
        actual_floors = {d for d in os.listdir(floors_parent_dir) if os.path.isdir(os.path.join(floors_parent_dir, d))}
        
        # 找出需要删除的键 (在config中但不在actual_floors里)
        keys_to_delete = [key for key in config_data if key not in actual_floors]
        
        if keys_to_delete:
            for key in keys_to_delete:
                del config_data[key]
                print(f"已清理无效条目: '{key}' (因为对应的文件夹不存在)")
        else:
            print("配置文件与目录结构一致，无需清理。")

    # 4. 将更新后的配置写回文件
    try:
        with open(config_path, 'w') as f:
            json.dump(config_data, f, indent=4) # indent=4 格式化输出，方便阅读
        print(f"已成功将更新后的配置保存到: {config_path}")
    except Exception as e:
        print(f"错误: 保存配置文件失败: {e}")

    print("--- z_config.json 管理完成 ---\n")


# ==============================================================================
#  点云处理与地图生成函数 (已优化文件保存结构)
# ==============================================================================

def filter_points_by_height(points, min_z, max_z):
    """根据Z轴高度过滤点云。"""
    in_range = np.logical_and(points[:, 2] >= min_z, points[:, 2] <= max_z)
    return points[in_range]

def pointcloud_to_occupancy_grid(pcd_file, floor_name, resolution, min_z, max_z, floors_parent_dir):
    """将PCD或PLY点云文件转换为ROS栅格地图和YAML配置文件。"""
    print(f"正在加载点云文件: {pcd_file}...")
    if not os.path.exists(pcd_file):
        print(f"错误: 输入文件不存在 -> {pcd_file}")
        return False
    try:
        pcd = o3d.io.read_point_cloud(pcd_file)
        if not pcd.has_points():
            print("错误: 点云文件中没有点。")
            return False
    except Exception as e:
        print(f"错误: 无法读取点云文件: {e}")
        return False

    points = np.asarray(pcd.points)
    print(f"点云加载成功，包含 {len(points)} 个点。")

    # 1. 高度过滤
    print(f"正在根据高度范围过滤点云 (min_z: {min_z} m, max_z: {max_z} m)...")
    filtered_points = filter_points_by_height(points, min_z, max_z)
    if len(filtered_points) == 0:
        print("错误: 在指定的高度范围内没有找到任何点。")
        return False
    print(f"过滤后剩余 {len(filtered_points)} 个点。")

    # 2. 计算地图尺寸
    x_coords, y_coords = filtered_points[:, 0], filtered_points[:, 1]
    x_min, x_max = np.min(x_coords), np.max(x_coords)
    y_min, y_max = np.min(y_coords), np.max(y_coords)
    map_width = int(np.ceil((x_max - x_min) / resolution))
    map_height = int(np.ceil((y_max - y_min) / resolution))
    print(f"计算出的地图尺寸: {map_width} x {map_height} 像素。")

    # 3. 创建栅格地图
    map_data = np.full((map_height, map_width), 205, dtype=np.uint8) # 205: 未知区域
    for point in filtered_points:
        mx = int((point[0] - x_min) / resolution)
        my = map_height - 1 - int((point[1] - y_min) / resolution)
        if 0 <= mx < map_width and 0 <= my < map_height:
            map_data[my, mx] = 0 # 0: 占据区域

    # 4. 准备保存路径 (优化后的结构)
    floor_save_folder = os.path.join(floors_parent_dir, floor_name)
    if not os.path.exists(floor_save_folder):
        print(f"创建楼层保存目录: {floor_save_folder}")
        os.makedirs(floor_save_folder)
        
    base_path = os.path.join(floor_save_folder, floor_name)

    # 5. 保存PGM地图文件
    pgm_file_path = f"{base_path}.pgm"
    print(f"正在保存PGM地图文件到: {pgm_file_path}")
    img = Image.fromarray(map_data, mode='L')
    img.save(pgm_file_path)

    # 6. 创建并保存YAML配置文件
    yaml_config = {
        'image': f"{floor_name}.pgm", # 相对路径
        'resolution': resolution,
        'origin': [float(x_min), float(y_min), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    yaml_file_path = f"{base_path}.yaml"
    print(f"正在保存YAML配置文件到: {yaml_file_path}")
    with open(yaml_file_path, 'w') as yaml_file:
        yaml.dump(yaml_config, yaml_file)

    print("\n地图生成完成！")
    print(f"地图文件: {pgm_file_path}")
    print(f"配置文件: {yaml_file_path}")
    
    return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='将PCD或PLY点云文件转换为ROS栅格地图，并管理z_config.json文件。')
    
    parser.add_argument('input_file', type=str, help='输入的点云文件路径 (.pcd or .ply)')
    parser.add_argument('--floor_name', type=str, default=None, 
                        help='指定生成的楼层名称。默认为输入文件的基本名称(不含扩展名)。')
    parser.add_argument('-r', '--resolution', type=float, default=0.05, help='地图分辨率 (米/像素)。默认为 0.05。')
    parser.add_argument('--min_z', type=float, default=0.1, help='高度过滤的最小值 (米)。默认为 0.1。')
    parser.add_argument('--max_z', type=float, default=2.0, help='高度过滤的最大值 (米)。默认为 2.0。')
    parser.add_argument('--floors_dir', type=str, default='resources/floors', help='存放所有楼层文件夹的基础目录。默认为 "resources/floors"。')

    args = parser.parse_args()

    # 如果未提供 floor_name，则从输入文件名自动生成
    if args.floor_name is None:
        floor_name_to_use = os.path.splitext(os.path.basename(args.input_file))[0]
        print(f"未指定 --floor_name，将使用输入文件的基本名称: '{floor_name_to_use}'")
    else:
        floor_name_to_use = args.floor_name

    # 执行地图生成
    success = pointcloud_to_occupancy_grid(
        pcd_file=args.input_file,
        floor_name=floor_name_to_use,
        resolution=args.resolution,
        min_z=args.min_z,
        max_z=args.max_z,
        floors_parent_dir=args.floors_dir
    )

    # 如果地图生成成功，则执行JSON配置管理
    if success:
        config_file_path = os.path.join(args.floors_dir, 'z_config.json')
        manage_z_config(
            config_path=config_file_path,
            current_floor=floor_name_to_use,
            min_z=args.min_z,
            max_z=args.max_z,
            floors_parent_dir=args.floors_dir
        )
    else:
        print("\n地图生成失败，跳过 z_config.json 的更新。")