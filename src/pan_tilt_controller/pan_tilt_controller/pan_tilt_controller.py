import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import math


class PanTiltController(Node):
    """云台控制器 - 用于扩展视觉系统视野"""
    
    def __init__(self):
        super().__init__('pan_tilt_controller')
        
        # 声明参数
        self.declare_parameter('enable_pan_tilt', True)
        self.declare_parameter('angular_vel_y', 10.0)
        self.declare_parameter('angular_vel_z', 45.0)
        self.declare_parameter('scan_range_y', 30.0)
        self.declare_parameter('scan_range_z', 60.0)
        self.declare_parameter('auto_scan', True)
        self.declare_parameter('scan_interval', 2.0)
        
        # 获取参数
        self.enable_pan_tilt = self.get_parameter('enable_pan_tilt').value
        self.angular_vel_y = self.get_parameter('angular_vel_y').value
        self.angular_vel_z = self.get_parameter('angular_vel_z').value
        self.scan_range_y = self.get_parameter('scan_range_y').value
        self.scan_range_z = self.get_parameter('scan_range_z').value
        self.auto_scan = self.get_parameter('auto_scan').value
        self.scan_interval = self.get_parameter('scan_interval').value
        
        # 当前云台状态
        self.current_angle_y = 0.0
        self.current_angle_z = 0.0
        self.target_angle_y = 0.0
        self.target_angle_z = 0.0
        self.is_scanning = False
        self.scan_step = 0
        
        # 订阅视觉目标检测结果
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/vision/target_pose',
            self.target_callback,
            10)
        
        # 发布云台控制指令（仿真环境）
        self.pan_tilt_pub = self.create_publisher(
            JointState,
            '/wpb_home/mani_ctrl',
            10)
        
        # 发布云台状态
        self.state_pub = self.create_publisher(
            PoseStamped,
            '/pan_tilt/state',
            10)
        
        # 创建定时器用于状态发布和扫描控制
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('PanTilt controller initialized')
        self.get_logger().info(f'Auto scan: {self.auto_scan}, Scan interval: {self.scan_interval}s')
    
    def target_callback(self, msg):
        """处理视觉目标检测结果 - 云台追踪目标"""
        if not self.enable_pan_tilt:
            return
        
        # 根据目标位置调整云台角度
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        
        # 简单的比例控制
        self.target_angle_z = -math.degrees(math.atan2(target_y, target_x)) * 0.5
        self.target_angle_y = math.degrees(math.atan2(target_y, target_x)) * 0.3
        
        # 限制角度范围
        self.target_angle_z = max(-self.scan_range_z, min(self.scan_range_z, self.target_angle_z))
        self.target_angle_y = max(-self.scan_range_y, min(self.scan_range_y, self.target_angle_y))
        
        self.get_logger().info(f'Target detected, adjusting pan-tilt to: y={self.target_angle_y:.2f}, z={self.target_angle_z:.2f}')
    
    def timer_callback(self):
        """定时器回调 - 状态发布和扫描控制"""
        # 发布云台状态
        self.publish_state()
        
        # 自动扫描模式
        if self.auto_scan and self.enable_pan_tilt and not self.is_scanning:
            self.start_scan()
        
        # 执行云台运动
        self.execute_movement()
    
    def start_scan(self):
        """开始扫描"""
        self.is_scanning = True
        self.scan_step = 0
        self.get_logger().info('Starting auto scan')
    
    def execute_movement(self):
        """执行云台运动"""
        if not self.enable_pan_tilt:
            return
        
        if self.is_scanning:
            # 自动扫描模式
            self.execute_scan()
        else:
            # 追踪模式 - 平滑移动到目标角度
            self.smooth_move_to_target()
        
        # 发布控制指令到仿真环境
        self.publish_to_sim()
    
    def execute_scan(self):
        """执行扫描序列"""
        # 简单的扫描模式：左右扫动
        scan_positions = [
            (0.0, 0.0),           # 中心
            (0.0, self.scan_range_z),   # 右
            (0.0, -self.scan_range_z),  # 左
            (self.scan_range_y, 0.0),   # 上
            (-self.scan_range_y, 0.0),  # 下
        ]
        
        if self.scan_step < len(scan_positions):
            self.target_angle_y, self.target_angle_z = scan_positions[self.scan_step]
            self.scan_step += 1
        else:
            self.is_scanning = False
            self.scan_step = 0
            self.get_logger().info('Scan completed')
    
    def smooth_move_to_target(self):
        """平滑移动到目标角度"""
        # 简单的比例控制
        self.current_angle_y += (self.target_angle_y - self.current_angle_y) * 0.1
        self.current_angle_z += (self.target_angle_z - self.current_angle_z) * 0.1
    
    def publish_to_sim(self):
        """发布控制指令到仿真环境"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['pan', 'tilt']
        
        # 转换角度到弧度
        msg.position = [
            math.radians(self.current_angle_z),  # pan (z轴)
            math.radians(self.current_angle_y)   # tilt (y轴)
        ]
        
        self.pan_tilt_pub.publish(msg)
    
    def publish_state(self):
        """发布云台状态"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'pan_tilt_link'
        
        # 将角度转换为位姿（简化表示）
        msg.pose.position.x = 0.0
        msg.pose.position.y = math.radians(self.current_angle_y)
        msg.pose.position.z = math.radians(self.current_angle_z)
        
        # 使用四元数表示旋转
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = math.sin(math.radians(self.current_angle_y) / 2)
        msg.pose.orientation.z = math.sin(math.radians(self.current_angle_z) / 2)
        
        self.state_pub.publish(msg)
    
    def set_pan_tilt_position(self, angle_y, angle_z):
        """设置云台位置"""
        self.target_angle_y = max(-self.scan_range_y, min(self.scan_range_y, angle_y))
        self.target_angle_z = max(-self.scan_range_z, min(self.scan_range_z, angle_z))
        self.get_logger().info(f'Setting pan-tilt to: y={angle_y:.2f}, z={angle_z:.2f}')
    
    def reset_position(self):
        """重置云台到中心位置"""
        self.target_angle_y = 0.0
        self.target_angle_z = 0.0
        self.get_logger().info('Resetting pan-tilt to center')


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()