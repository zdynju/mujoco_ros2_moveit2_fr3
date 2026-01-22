import numpy as np
from scipy.spatial.transform import Rotation as R

def mujoco_xyaxes_to_quat(xyaxes_str):
    # 1. 解析 XML 里的字符串
    vals = [float(x) for x in xyaxes_str.split()]
    
    # 2. 提取 X 轴 (相机右方向) 和 Y 轴 (相机上方向)
    x_axis = np.array(vals[0:3]) # [0.707, -0.707, 0]
    y_axis = np.array(vals[3:6]) # [-0.408, -0.408, 0.816]
    
    # 3. 归一化 (防止手写的数不标准)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    
    # 4. 计算 Z 轴 (相机前方) = X 叉乘 Y (右手定则)
    # MuJoCo 的定义是：Z = Right cross Up ??? 
    # 不，MuJoCo 的相机坐标系是 -Z 看向前方。
    # 但 ROS 的 Optical Frame 定义是 Z 向前，X 向右，Y 向下。
    # 这里有一个巨大的坑：MuJoCo 的 xyaxes 定义的是屏幕的 X 和 Y。
    # 对应的 ROS Optical Frame (Z-forward) 应该是：
    # ROS_X = MuJoCo_X (Right)
    # ROS_Y = -MuJoCo_Y (Down)  <-- 注意这里，屏幕向上等于相机Y轴向下
    # ROS_Z = Cross(ROS_X, ROS_Y) (Forward)
    
    ros_x = x_axis
    ros_y = -y_axis # 屏幕向上，意味着光轴坐标系的Y轴是负的
    ros_z = np.cross(ros_x, ros_y)
    
    # 5. 构建旋转矩阵
    rot_mat = np.column_stack((ros_x, ros_y, ros_z))
    
    # 6. 转为四元数
    r = R.from_matrix(rot_mat)
    return r.as_quat()

# 你的 XML 数据
xml_xyaxes = "0.707 -0.707 0 0.408 0.408 0.816"
xml_pos = "-2.8 -2.8 2.1"

q = mujoco_xyaxes_to_quat(xml_xyaxes)
pos = xml_pos.split()

print("✅ 复制这一段到你的 Launch 文件:")
print(f"arguments=['--x', '{pos[0]}', '--y', '{pos[1]}', '--z', '{pos[2]}',")
print(f"           '--qx', '{q[0]:.4f}', '--qy', '{q[1]:.4f}', '--qz', '{q[2]:.4f}', '--qw', '{q[3]:.4f}',")
print(f"           '--frame-id', 'world', '--child-frame-id', 'world_camera_optical_frame']")