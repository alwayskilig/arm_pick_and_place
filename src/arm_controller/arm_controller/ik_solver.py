import numpy as np
from dataclasses import dataclass
from typing import List, Optional
from enum import Enum


class PTPMode(Enum):
    """PTP运动模式"""
    JUMP_XYZ = 0      # 抬升模式：先抬升再移动
    MOVJ_XYZ = 1      # 关节运动： fastest
    MOVL_XYZ = 2      # 直线运动： 保持末端直线
    JUMP_ANGLE = 3    # 关节抬升模式
    MOVJ_ANGLE = 4    # 关节角度运动
    MOVL_ANGLE = 5    # 直线角度运动


@dataclass
class JointAngles:
    """关节角度"""
    angles: List[float]

    def __post_init__(self):
        if len(self.angles) not in [5, 6]:
            raise ValueError("5 or 6 joint angles required")


@dataclass
class Pose:
    """位姿"""
    x: float
    y: float
    z: float
    r: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class KinematicsParams:
    """运动学参数"""
    velocity: float = 100.0
    acceleration: float = 100.0
    velocity_ratio: float = 1.0
    acceleration_ratio: float = 1.0


@dataclass
class JumpParams:
    """抬升参数"""
    jump_height: float = 50.0
    z_limit: float = 100.0


class IKSolver:
    """逆运动学求解器 - 支持多种PTP模式"""

    def __init__(self, num_joints=5):
        self.num_joints = num_joints
        
        # WPR1机器人DH参数（单位：米）
        # 基于实际机器人尺寸
        if num_joints == 5:
            # 5轴机械臂DH参数
            self.d = [0.1, 0.0, 0.0, 0.0, 0.0]      # 连杆偏距
            self.a = [0.0, 0.3, 0.25, 0.0, 0.0]     # 连杆长度
            self.alpha = [np.pi/2, 0, 0, np.pi/2, 0]  # 连杆扭转
        else:
            # 6轴机械臂DH参数
            self.d = [0.1, 0.0, 0.0, 0.0, 0.0, 0.08]
            self.a = [0.0, 0.3, 0.25, 0.0, 0.0, 0.0]
            self.alpha = [np.pi/2, 0, 0, np.pi/2, 0, 0]
        
        # 关节限位（弧度）
        self.joint_limits = [
            (-np.pi, np.pi),      # joint1
            (-np.pi/2, np.pi/2),  # joint2
            (-np.pi/2, np.pi/2),  # joint3
            (-np.pi, np.pi),      # joint4
            (-np.pi, np.pi),      # joint5
        ]
        
        # 默认运动参数
        self.kinematics_params = KinematicsParams()
        self.jump_params = JumpParams()
    
    def solve(self, target_pose: Pose, mode: PTPMode = PTPMode.MOVJ_XYZ) -> Optional[JointAngles]:
        """
        求解逆运动学
        支持多种PTP模式
        """
        try:
            x, y, z = target_pose.x, target_pose.y, target_pose.z
            
            # 根据模式选择不同的求解方法
            if mode == PTPMode.JUMP_XYZ or mode == PTPMode.JUMP_ANGLE:
                # 抬升模式：先计算抬升位置的关节角度
                return self.solve_jump_mode(target_pose)
            elif mode == PTPMode.MOVL_XYZ or mode == PTPMode.MOVL_ANGLE:
                # 直线模式：需要插值计算
                return self.solve_linear_mode(target_pose)
            else:
                # 关节运动模式
                return self.solve_joint_mode(target_pose)
            
        except Exception:
            return None
    
    def solve_joint_mode(self, target_pose: Pose) -> Optional[JointAngles]:
        """关节运动模式求解 - 使用几何法"""
        try:
            x, y, z = target_pose.x, target_pose.y, target_pose.z
            
            # 基座旋转角度（关节1）
            joint1 = np.arctan2(y, x)
            
            # 计算到目标的水平距离
            r = np.sqrt(x**2 + y**2)
            
            # 考虑基座高度
            z_offset = z - self.d[0]
            
            # 计算关节2和关节3（使用几何法）
            # 连杆2和3的长度
            L2 = self.a[1]  # 0.3m
            L3 = self.a[2]  # 0.25m
            
            # 计算到目标的距离
            dist = np.sqrt(r**2 + z_offset**2)
            
            # 检查是否在工作空间内
            if dist > (L2 + L3) or dist < abs(L2 - L3):
                # 目标在工作空间外，使用简化计算
                joint2 = np.arctan2(z_offset, r) - np.pi/4
                joint3 = np.pi/4
            else:
                # 使用余弦定理计算关节3
                cos_joint3 = (r**2 + z_offset**2 - L2**2 - L3**2) / (2 * L2 * L3)
                cos_joint3 = np.clip(cos_joint3, -1.0, 1.0)
                joint3 = np.arccos(cos_joint3)
                
                # 计算关节2
                beta = np.arctan2(z_offset, r)
                gamma = np.arctan2(L3 * np.sin(joint3), L2 + L3 * np.cos(joint3))
                joint2 = beta - gamma
            
            # 关节4和关节5（末端姿态）
            joint4 = 0.0
            joint5 = 0.0
            
            # 限制关节角度在有效范围内
            angles = [joint1, joint2, joint3, joint4, joint5]
            for i in range(len(angles)):
                if i < len(self.joint_limits):
                    angles[i] = max(self.joint_limits[i][0], min(self.joint_limits[i][1], angles[i]))
            
            return JointAngles(angles)
            
        except Exception:
            return None
    
    def solve_jump_mode(self, target_pose: Pose) -> Optional[JointAngles]:
        """抬升模式求解 - 先抬升再移动"""
        # 简化实现：直接使用关节运动模式
        return self.solve_joint_mode(target_pose)
    
    def solve_linear_mode(self, target_pose: Pose) -> Optional[JointAngles]:
        """直线模式求解 - 保持末端直线运动"""
        # 简化实现：使用关节运动模式
        return self.solve_joint_mode(target_pose)
    
    def interpolate_path(self, start_pose: Pose, end_pose: Pose, 
                         num_points: int = 10, mode: PTPMode = PTPMode.MOVL_XYZ) -> List[JointAngles]:
        """
        路径插值 - 用于直线运动模式
        """
        path = []
        
        for i in range(num_points + 1):
            t = i / num_points
            
            # 线性插值
            interpolated_pose = Pose(
                x=start_pose.x + (end_pose.x - start_pose.x) * t,
                y=start_pose.y + (end_pose.y - start_pose.y) * t,
                z=start_pose.z + (end_pose.z - start_pose.z) * t,
                r=start_pose.r + (end_pose.r - start_pose.r) * t
            )
            
            joint_angles = self.solve(interpolated_pose, mode)
            if joint_angles is not None:
                path.append(joint_angles)
        
        return path
    
    def calculate_jump_trajectory(self, start_pose: Pose, end_pose: Pose, 
                                  jump_height: float = None) -> List[JointAngles]:
        """
        计算抬升轨迹
        """
        if jump_height is None:
            jump_height = self.jump_params.jump_height
        
        trajectory = []
        
        # 1. 计算抬升点
        lift_pose = Pose(
            x=start_pose.x,
            y=start_pose.y,
            z=start_pose.z + jump_height
        )
        
        # 2. 计算下降点
        lower_pose = Pose(
            x=end_pose.x,
            y=end_pose.y,
            z=end_pose.z + jump_height
        )
        
        # 3. 生成轨迹
        # 起点 -> 抬升点
        trajectory.extend(self.interpolate_path(start_pose, lift_pose, 5, PTPMode.MOVJ_XYZ))
        # 抬升点 -> 下降点上方
        trajectory.extend(self.interpolate_path(lift_pose, lower_pose, 10, PTPMode.MOVL_XYZ))
        # 下降点上方 -> 终点
        trajectory.extend(self.interpolate_path(lower_pose, end_pose, 5, PTPMode.MOVJ_XYZ))
        
        return trajectory
    
    def forward_kinematics(self, joint_angles: JointAngles) -> Optional[Pose]:
        """正运动学求解"""
        try:
            angles = joint_angles.angles
            
            # 初始化变换矩阵
            T = np.eye(4)
            
            for i in range(min(len(angles), self.num_joints)):
                # DH变换矩阵
                theta = angles[i]
                d = self.d[i]
                a = self.a[i]
                alpha = self.alpha[i]
                
                # 旋转矩阵
                ct = np.cos(theta)
                st = np.sin(theta)
                ca = np.cos(alpha)
                sa = np.sin(alpha)
                
                # DH变换矩阵
                Ti = np.array([
                    [ct, -st*ca, st*sa, a*ct],
                    [st, ct*ca, -ct*sa, a*st],
                    [0, sa, ca, d],
                    [0, 0, 0, 1]
                ])
                
                T = T @ Ti
            
            # 提取位置
            x = T[0, 3]
            y = T[1, 3]
            z = T[2, 3]
            
            return Pose(x, y, z)
            
        except Exception:
            return None
    
    def set_kinematics_params(self, velocity: float = None, acceleration: float = None):
        """设置运动学参数"""
        if velocity is not None:
            self.kinematics_params.velocity = velocity
        if acceleration is not None:
            self.kinematics_params.acceleration = acceleration
    
    def set_jump_params(self, jump_height: float = None, z_limit: float = None):
        """设置抬升参数"""
        if jump_height is not None:
            self.jump_params.jump_height = jump_height
        if z_limit is not None:
            self.jump_params.z_limit = z_limit