import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Bool
from arm_pick_and_place.msg import TaskCommand, TaskStatus
from .ik_solver import IKSolver, Pose as IKPose, PTPMode
import time
import math


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self.declare_parameter('arm_joint_names', [
            'arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5'
        ])
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('max_acceleration', 1.0)
        self.declare_parameter('home_position', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('pick_height', 0.1)
        self.declare_parameter('place_height', 0.1)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('use_jump_mode', True)
        self.declare_parameter('jump_height', 0.05)

        self.joint_names = self.get_parameter('arm_joint_names').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.home_position = self.get_parameter('home_position').value
        self.pick_height = self.get_parameter('pick_height').value
        self.place_height = self.get_parameter('place_height').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.use_jump_mode = self.get_parameter('use_jump_mode').value
        self.jump_height = self.get_parameter('jump_height').value

        self.num_joints = len(self.joint_names)
        self.ik_solver = IKSolver(num_joints=self.num_joints)
        
        # 设置运动参数
        self.ik_solver.set_jump_params(jump_height=self.jump_height)

        self.current_joints = [0.0] * self.num_joints
        self.target_joints = [0.0] * self.num_joints
        self.is_moving = False
        self.goal_reached = True

        self.target_sub = self.create_subscription(
            PoseStamped,
            '/vision/target_pose',
            self.target_callback,
            10)

        self.command_sub = self.create_subscription(
            TaskCommand,
            '/task/command',
            self.command_callback,
            10)

        self.joint_pub = self.create_publisher(
            JointState,
            '/arm/joint_states',
            10)

        self.status_pub = self.create_publisher(
            TaskStatus,
            '/task/status',
            10)
        
        # 目标到达状态发布
        self.goal_reached_pub = self.create_publisher(
            Bool,
            '/arm/goal_reached',
            10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Arm controller initialized')
        self.get_logger().info(f'Jump mode: {self.use_jump_mode}, Jump height: {self.jump_height}')

    def timer_callback(self):
        """定时器回调 - 发布状态和检测目标到达"""
        self.publish_joint_state()
        self.check_goal_reached()
    
    def target_callback(self, msg):
        if self.is_moving:
            return

        self.get_logger().info(f'Received target pose: {msg.pose.position}')

        target = IKPose(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z
        )
        
        # 选择PTP模式
        mode = PTPMode.JUMP_XYZ if self.use_jump_mode else PTPMode.MOVJ_XYZ

        joint_angles = self.ik_solver.solve(target, mode)

        if joint_angles is not None:
            self.move_to_joints(joint_angles.angles)

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.command_type}')

        if msg.command_type == TaskCommand.RESET:
            self.move_home()
        elif msg.command_type == TaskCommand.GRASP:
            self.move_to_grasp(msg.target_pose)
        elif msg.command_type == TaskCommand.DELIVER:
            self.move_to_deliver(msg.target_pose)

    def move_to_joints(self, target_joints, duration=2.0):
        self.is_moving = True
        self.goal_reached = False
        self.target_joints = target_joints.copy()

        self.publish_status(TaskStatus.MOVING, 0.0, 'Moving to target')

        start_joints = self.current_joints.copy()
        start_time = time.time()

        while time.time() - start_time < duration:
            elapsed = time.time() - start_time
            progress = min(elapsed / duration, 1.0)
            
            # 使用平滑插值（梯形速度曲线）
            smooth_progress = self.trapezoidal_profile(progress)

            for i in range(self.num_joints):
                self.current_joints[i] = start_joints[i] + \
                    (target_joints[i] - start_joints[i]) * smooth_progress

            time.sleep(0.01)

        self.current_joints = target_joints.copy()
        self.is_moving = False
        self.goal_reached = True

        self.publish_status(TaskStatus.IDLE, 1.0, 'Movement complete')
        self.publish_goal_reached(True)
    
    def trapezoidal_profile(self, t):
        """梯形速度曲线"""
        # 加速阶段、匀速阶段、减速阶段
        if t < 0.2:
            # 加速阶段
            return 2.5 * t * t
        elif t < 0.8:
            # 匀速阶段
            return 0.1 + (t - 0.2)
        else:
            # 减速阶段
            return 0.7 + 2.5 * (0.8 - t) * (0.8 - t) + 1.6 * (t - 0.8)
    
    def check_goal_reached(self):
        """检测目标是否到达"""
        if self.is_moving:
            return
        
        # 计算当前位置与目标位置的距离
        distance = 0.0
        for i in range(self.num_joints):
            distance += (self.current_joints[i] - self.target_joints[i]) ** 2
        distance = math.sqrt(distance)
        
        # 检查是否在容差范围内
        if distance < self.position_tolerance and not self.goal_reached:
            self.goal_reached = True
            self.get_logger().info(f'Goal reached! Distance: {distance:.4f}')

    def move_home(self):
        self.get_logger().info('Moving to home position')
        self.move_to_joints(self.home_position)

    def move_to_grasp(self, target_pose):
        self.get_logger().info('Moving to grasp position')
        
        # 先移动到物体上方
        above_pose = IKPose(
            x=target_pose.pose.position.x,
            y=target_pose.pose.position.y,
            z=target_pose.pose.position.z + self.pick_height
        )
        
        mode = PTPMode.JUMP_XYZ if self.use_jump_mode else PTPMode.MOVJ_XYZ
        joint_angles = self.ik_solver.solve(above_pose, mode)
        if joint_angles is not None:
            self.move_to_joints(joint_angles.angles)
        
        # 等待到达
        time.sleep(0.5)
        
        # 下降到抓取位置
        grasp_pose = IKPose(
            x=target_pose.pose.position.x,
            y=target_pose.pose.position.y,
            z=target_pose.pose.position.z
        )
        
        joint_angles = self.ik_solver.solve(grasp_pose, PTPMode.MOVL_XYZ)
        if joint_angles is not None:
            self.move_to_joints(joint_angles.angles, duration=1.0)

    def move_to_deliver(self, target_pose):
        self.get_logger().info('Moving to deliver position')
        
        # 先抬升
        lift_pose = IKPose(
            x=self.current_joints[0],  # 保持当前位置
            y=self.current_joints[1],
            z=self.pick_height
        )
        
        mode = PTPMode.JUMP_XYZ if self.use_jump_mode else PTPMode.MOVJ_XYZ
        joint_angles = self.ik_solver.solve(lift_pose, mode)
        if joint_angles is not None:
            self.move_to_joints(joint_angles.angles)
        
        time.sleep(0.5)
        
        # 移动到递送点上方
        above_pose = IKPose(
            x=target_pose.pose.position.x,
            y=target_pose.pose.position.y,
            z=target_pose.pose.position.z + self.place_height
        )
        
        joint_angles = self.ik_solver.solve(above_pose, mode)
        if joint_angles is not None:
            self.move_to_joints(joint_angles.angles)
        
        time.sleep(0.5)
        
        # 下降到放置位置
        deliver_pose = IKPose(
            x=target_pose.pose.position.x,
            y=target_pose.pose.position.y,
            z=target_pose.pose.position.z
        )
        
        joint_angles = self.ik_solver.solve(deliver_pose, PTPMode.MOVL_XYZ)
        if joint_angles is not None:
            self.move_to_joints(joint_angles.angles, duration=1.0)

    def publish_joint_state(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_joints
        msg.velocity = []
        msg.effort = []

        self.joint_pub.publish(msg)

    def publish_status(self, state, progress, message):
        msg = TaskStatus()
        msg.state = state
        msg.progress = progress
        msg.message = message

        self.status_pub.publish(msg)
    
    def publish_goal_reached(self, reached):
        """发布目标到达状态"""
        msg = Bool()
        msg.data = reached
        self.goal_reached_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
