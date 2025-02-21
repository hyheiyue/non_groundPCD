import open3d as o3d
import numpy as np
import json  

def extract_ground_with_z_range(pcd, params):
    """
    提取z轴范围在[z_min, z_max]内且坡度小于max_slope_deg的地面点云
    :param pcd: 输入点云（需为numpy数组或Open3D点云）
    :param params: 参数字典，包含z_min, z_max, max_slope_deg, max_attempts, distance_threshold
    :return: 地面点云, 非地面点云
    """
    z_min = params['z_min']
    z_max = params['z_max']
    max_slope_deg = params['max_slope_deg']
    max_attempts = params['max_attempts']
    distance_threshold = params['distance_threshold']

    if not isinstance(pcd, o3d.geometry.PointCloud):
        pcd = o3d.geometry.PointCloud(pcd)
        
    # 预处理：根据z轴范围快速裁剪候选区域（加速后续计算）
    bbox = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=[-np.inf, -np.inf, z_min],
        max_bound=[np.inf, np.inf, z_max]
    )
    cropped_pcd = pcd.crop(bbox)
    
    # 初始化结果容器
    all_ground = o3d.geometry.PointCloud()
    remaining_cloud = cropped_pcd
    
    # 计算坡度对应的法线z分量阈值
    slope_rad = np.deg2rad(max_slope_deg)
    normal_z_min = np.cos(slope_rad)  # 例如15°对应0.966
    
    for _ in range(max_attempts):
        # 1. RANSAC拟合平面
        plane_model, inliers = remaining_cloud.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=3,
            num_iterations=1000
        )
        
        if len(inliers) == 0:
            break
            
        # 2. 提取候选地面点
        ground_candidate = remaining_cloud.select_by_index(inliers)
        points = np.asarray(ground_candidate.points)
        
        # 3. 双重验证：坡度 + 高度范围
        a, b, c, d = plane_model
        normal = np.array([a, b, c])
        normal /= np.linalg.norm(normal)
        normal_z = abs(normal[2])
        
        # 计算候选点平均高度
        avg_z = np.mean(points[:, 2])
        
        if normal_z >= normal_z_min and (z_min <= avg_z <= z_max):
            # 通过验证，加入地面点云
            all_ground += ground_candidate
            # 更新剩余点云
            remaining_cloud = remaining_cloud.select_by_index(inliers, invert=True)
        else:
            # 移除当前候选点（非地面干扰）
            remaining_cloud = remaining_cloud.select_by_index(inliers, invert=True)
    
    # 合并原始点云中超出z范围的非地面点
    points = np.asarray(pcd.points)
    non_ground_mask = (points[:, 2] < z_min) | (points[:, 2] > z_max)
    non_ground_points = points[non_ground_mask]
    non_ground = o3d.geometry.PointCloud()
    non_ground.points = o3d.utility.Vector3dVector(non_ground_points)
    non_ground += remaining_cloud
    
    return all_ground, non_ground


def project_to_ground_plane(non_ground_cloud, ground_cloud):
    """
    将非地面点云投影到地面平面
    :param non_ground_cloud: 非地面点云
    :param ground_cloud: 地面点云
    :return: 投影后的点云
    """
    if len(ground_cloud.points) == 0:
        print("警告：地面点云为空，无法投影")
        return None
    
    # 将地面点云转换为numpy数组
    ground_points = np.asarray(ground_cloud.points)
    
    # 计算质心和协方差矩阵
    centroid = np.mean(ground_points, axis=0)
    cov_matrix = np.cov(ground_points - centroid, rowvar=False)
    
    # 计算特征值和特征向量
    eigen_values, eigen_vectors = np.linalg.eigh(cov_matrix)
    
    # 最小特征值对应的特征向量为法向量
    normal = eigen_vectors[:, 0]
    
    # 确保法向量朝上（调整方向）
    if normal[2] < 0:
        normal = -normal
    
    a, b, c = normal
    d = -np.dot(normal, centroid)
    
    # 获取非地面点
    non_ground_points = np.asarray(non_ground_cloud.points)
    if len(non_ground_points) == 0:
        print("警告：非地面点云为空")
        return None
    
    # 计算每个点的投影
    projected_points = []
    for p in non_ground_points:
        x, y, z = p
        distance = a * x + b * y + c * z + d
        px = x - a * distance
        py = y - b * distance
        pz = z - c * distance
        projected_points.append([px, py, pz])
    
    # 创建投影后的点云
    projected_pcd = o3d.geometry.PointCloud()
    projected_pcd.points = o3d.utility.Vector3dVector(np.array(projected_points))
    
    return projected_pcd



# 使用示例 --------------------------------------------------
with open('params.json', 'r') as f:
    params = json.load(f)

# 从参数文件中获取输入路径
input_path = params['input_path']
pcd = o3d.io.read_point_cloud(input_path)

# 执行地面分割
ground_cloud, non_ground_cloud = extract_ground_with_z_range(pcd, params)

# 保存地面和非地面点云
o3d.io.write_point_cloud("ground.pcd", ground_cloud)
o3d.io.write_point_cloud("non_ground.pcd", non_ground_cloud)


if params.get('use_projection', False):  # 默认为 False
    projected_non_ground = project_to_ground_plane(non_ground_cloud, ground_cloud)

    if projected_non_ground is not None:
        # 保存投影后的点云
        o3d.io.write_point_cloud("non_ground_projected.pcd", projected_non_ground)
        print("投影完成并保存为 non_ground_projected.pcd")
        
        # 可视化
        ground_cloud.paint_uniform_color([0, 1, 0])        # 绿色为地面
        non_ground_cloud.paint_uniform_color([1, 0, 0])    # 红色为非地面原数据
        projected_non_ground.paint_uniform_color([0, 0, 1])  # 蓝色表示投影点
        o3d.visualization.draw_geometries([ground_cloud, non_ground_cloud, projected_non_ground])
    else:
        print("投影失败")
