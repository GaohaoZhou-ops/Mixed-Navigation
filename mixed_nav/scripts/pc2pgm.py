import open3d as o3d
import numpy as np
import yaml
from PIL import Image
import os
import argparse

def filter_points_by_height(points, min_z, max_z):
    """根据Z轴高度过滤点云。"""
    in_range = np.logical_and(points[:, 2] >= min_z, points[:, 2] <= max_z)
    return points[in_range]

def pointcloud_to_occupancy_grid(pcd_file, output_map_name, resolution, min_z, max_z, save_folder):
    """将PCD或PLY点云文件转换为ROS栅格地图和YAML配置文件。"""
    print(f"正在加载点云文件: {pcd_file}...")
    if not os.path.exists(pcd_file):
        print(f"错误: 输入文件不存在 -> {pcd_file}")
        return
    try:
        pcd = o3d.io.read_point_cloud(pcd_file)
        if not pcd.has_points():
            print("错误: 点云文件中没有点。")
            return
    except Exception as e:
        print(f"错误: 无法读取点云文件: {e}")
        return

    points = np.asarray(pcd.points)
    print(f"点云加载成功，包含 {len(points)} 个点。")

    # 1. 高度过滤
    print(f"正在根据高度范围过滤点云 (min_z: {min_z} m, max_z: {max_z} m)...")
    filtered_points = filter_points_by_height(points, min_z, max_z)
    if len(filtered_points) == 0:
        print("错误: 在指定的高度范围内没有找到任何点。")
        return
    print(f"过滤后剩余 {len(filtered_points)} 个点。")

    # 2. 计算地图尺寸
    x_coords = filtered_points[:, 0]
    y_coords = filtered_points[:, 1]
    x_min, x_max = np.min(x_coords), np.max(x_coords)
    y_min, y_max = np.min(y_coords), np.max(y_coords)

    map_width = int(np.ceil((x_max - x_min) / resolution))
    map_height = int(np.ceil((y_max - y_min) / resolution))
    print(f"计算出的地图尺寸: {map_width} x {map_height} 像素。")

    # 3. 创建栅格地图
    map_data = np.full((map_height, map_width), 205, dtype=np.uint8)
    
    for point in filtered_points:
        mx = int((point[0] - x_min) / resolution)
        my = map_height - 1 - int((point[1] - y_min) / resolution)

        if 0 <= mx < map_width and 0 <= my < map_height:
            map_data[my, mx] = 0

    # 4. 准备保存路径
    if not os.path.exists(save_folder):
        print(f"创建保存目录: {save_folder}")
        os.makedirs(save_folder)
        
    base_path = os.path.join(save_folder, output_map_name)

    # 5. 保存PGM地图文件
    pgm_file_path = f"{base_path}.pgm"
    print(f"正在保存PGM地图文件到: {pgm_file_path}")
    img = Image.fromarray(map_data, mode='L')
    img.save(pgm_file_path)

    # 6. 创建并保存YAML配置文件
    origin_x = float(x_min)
    origin_y = float(y_min)
    
    yaml_config = {
        'image': os.path.basename(pgm_file_path),
        'resolution': resolution,
        'origin': [origin_x, origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }

    yaml_file_path = f"{base_path}.yaml"
    print(f"正在保存YAML配置文件到: {yaml_file_path}")
    with open(yaml_file_path, 'w') as yaml_file:
        yaml.dump(yaml_config, yaml_file)

    print("\n处理完成！")
    print(f"地图文件: {pgm_file_path}")
    print(f"配置文件: {yaml_file_path}")
    print("\n您现在可以使用ROS map_server加载此地图:")
    print(f"rosrun map_server map_server {os.path.abspath(yaml_file_path)}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='将PCD或PLY点云文件转换为ROS栅格地图。')
    
    parser.add_argument('input_file', type=str, help='输入的点云文件路径 (.pcd or .ply)')
    parser.add_argument(
        '--map_name', 
        type=str, 
        default=None, 
        help='指定生成的地图名称。默认为输入文件名前加 "2d_" 前缀。'
    )
    
    parser.add_argument('-r', '--resolution', type=float, default=0.05, help='地图分辨率 (米/像素)。默认为 0.05。')
    parser.add_argument('--min_z', type=float, default=0.1, help='高度过滤的最小值 (米)。默认为 0.1。')
    parser.add_argument('--max_z', type=float, default=2.0, help='高度过滤的最大值 (米)。默认为 2.0。')
    parser.add_argument('--save_folder', type=str, default='resources', help='保存生成的地图文件的文件夹。默认为 "resources"。')

    args = parser.parse_args()

    # 处理 map_name 的默认值
    if args.map_name is None:
        # 获取输入文件的基础名称（不含路径和扩展名）
        base_name = os.path.splitext(os.path.basename(args.input_file))[0]
        # 设置默认地图名称
        map_name_to_use = f"2d_{base_name}"
        print(f"未指定 --map_name，使用默认名称: {map_name_to_use}")
    else:
        map_name_to_use = args.map_name

    pointcloud_to_occupancy_grid(
        pcd_file=args.input_file,
        output_map_name=map_name_to_use, # 使用处理后的地图名称
        resolution=args.resolution,
        min_z=args.min_z,
        max_z=args.max_z,
        save_folder=args.save_folder
    )