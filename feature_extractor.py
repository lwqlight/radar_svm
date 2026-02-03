import numpy as np

# 定义特征列名 (共 10 维)
FEATURE_NAMES = [
    "target_z",      # 目标高度 (来自 0x0A04)
    "target_speed",  # 目标速度 (来自 0x0A04)
    "cloud_width",   # 点云宽度 (X轴跨度)
    "cloud_depth",   # 点云深度 (Y轴跨度)
    "cloud_area",    # 投影面积 (宽 * 深)
    "cloud_ratio",   # 长宽比 (宽 / 深) -> 区分站立/躺下的神器
    "cloud_std_x",   # X轴离散度 (标准差)
    "cloud_std_y",   # Y轴离散度 (标准差)
    "cloud_count",   # 点数
    "cloud_density"  # 密度 (点数 / 面积)
]

def extract_features(target_info, point_cloud_list):
    """
    输入:
      target_info: 字典 {'z': float, 'speed': float}
      point_cloud_list: 列表 [(x, y, speed), ...]
    输出:
      10维特征列表
    """
    # 1. 基础目标特征
    base_feats = [
        target_info.get('z', 0.0),
        target_info.get('speed', 0.0)
    ]
    
    # 2. 点云几何特征
    if len(point_cloud_list) >= 3:
        # 转为 numpy 数组: [[x, y], ...]
        pts = np.array([[p[0], p[1]] for p in point_cloud_list])
        
        # 计算包围盒
        x_min, y_min = np.min(pts, axis=0)
        x_max, y_max = np.max(pts, axis=0)
        
        width = x_max - x_min
        depth = y_max - y_min
        area = width * depth
        
        # 防止除以0
        ratio = width / (depth + 0.001)
        
        # 计算离散度 (标准差)
        std_x = np.std(pts[:, 0])
        std_y = np.std(pts[:, 1])
        
        # 计算密度
        density = len(point_cloud_list) / (area + 0.01)
        
        cloud_feats = [
            width, depth, area, ratio, 
            std_x, std_y, 
            len(point_cloud_list), density
        ]
    else:
        # 点太少，几何特征无效，给默认值
        cloud_feats = [0.0] * 8
        
    return base_feats + cloud_feats