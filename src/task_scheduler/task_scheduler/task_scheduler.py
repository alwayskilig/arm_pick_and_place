import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from arm_pick_and_place.msg import TaskCommand, TaskStatus
from .states import TaskState
import time


class TaskScheduler(Node):
    def __init__(self):
        super().__init__('task_scheduler')

        # 声明参数
        self.declare_parameter('pick_position', [0.3, 0.0, 0.1])
        self.declare_parameter('place_position', [-0.3, 0.0, 0.1])
        self.declare_parameter('safe_height', 0.3)
        self.declare_parameter('move_timeout', 10.0)

        # 获取参数
        self.pick_position = self.get_parameter('pick_position').value
        self.place_position = self.get_parameter('place_position').value
        self.safe_height = self.get_parameter('safe_height').value
        self.move_timeout = self.get_parameter('move_timeout').value

        # 状态机
        self.current_state = TaskState.IDLE
        self.target_pose = None
        self.grasp_pose = None

        # 订阅视觉目标
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/vision/target_pose',
            self.target_callback,
            10)

        # 订阅关节状态
        self.joint_sub = self.create_subscription(
            JointState,
            '/arm/joint_states',
            self.joint_callback,
            10)

        # 订阅夹爪状态
        self.gripper_sub = self.create_subscription(
            Float64,
            '/gripper/state',
            self.gripper_callback,
            10)

        # 发布任务指令
        self.command_pub = self.create_publisher(
            TaskCommand,
            '/task/command',
            10)

        # 发布任务状态
        self.status_pub = self.create_publisher(
            TaskStatus,
            '/task/status',
            10)

        # 创建状态机定时器
        self.timer = self.create_timer(0.1, self.state_machine)

        self.get_logger().info('Task scheduler initialized')

    def target_callback(self, msg):
        """处理视觉目标"""
        if self.current_state == TaskState.DETECTING:
            self.target_pose = msg
            self.get_logger().info(f'Target detected: {msg.pose.position}')

    def joint_callback(self, msg):
        """处理关节状态"""
        pass

    def gripper_callback(self, msg):
        """处理夹爪状态"""
        pass

    def state_machine(self):
        """状态机主循环"""
        if self.current_state == TaskState.IDLE:
            pass

        elif self.current_state == TaskState.DETECTING:
            if self.target_pose is not None:
                self.get_logger().info('Target found, moving to object')
                self.current_state = TaskState.MOVING_TO_OBJECT
                self.send_grasp_command()

        elif self.current_state == TaskState.MOVING_TO_OBJECT:
            # 等待移动完成
            pass

        elif self.current_state == TaskState.GRASPING:
            # 等待抓取完成
            pass

        elif self.current_state == TaskState.LIFTING:
            # 等待提升完成
            pass

        elif self.current_state == TaskState.MOVING_TO_TARGET:
            # 等待移动到递送点完成
            pass

        elif self.current_state == TaskState.RELEASING:
            # 等待释放完成
            pass

        elif self.current_state == TaskState.RETURNING:
            # 等待返回零位完成
            pass

        elif self.current_state == TaskState.ERROR:
            pass

    def send_grasp_command(self):
        """发送抓取指令"""
        msg = TaskCommand()
        msg.command_type = TaskCommand.GRASP

        if self.target_pose is not None:
            msg.target_pose = self.target_pose

        self.command_pub.publish(msg)
        self.publish_status(TaskState.GRASPING, 0.0, 'Grasping object')

    def send_deliver_command(self):
        """发送递送指令"""
        msg = TaskCommand()
        msg.command_type = TaskCommand.DELIVER

        if self.target_pose is not None:
            msg.target_pose = self.target_pose

        self.command_pub.publish(msg)
        self.publish_status(TaskState.RELEASING, 0.0, 'Delivering object')

    def send_reset_command(self):
        """发送复位指令"""
        msg = TaskCommand()
        msg.command_type = TaskCommand.RESET
        self.command_pub.publish(msg)
        self.publish_status(TaskState.RETURNING, 0.0, 'Returning to home')

    def publish_status(self, state, progress, message):
        """发布任务状态"""
        msg = TaskStatus()

        # 映射状态
        state_map = {
            TaskState.IDLE: TaskStatus.IDLE,
            TaskState.DETECTING: TaskStatus.MOVING,
            TaskState.MOVING_TO_OBJECT: TaskStatus.MOVING,
            TaskState.GRASPING: TaskStatus.GRASPING,
            TaskState.LIFTING: TaskStatus.MOVING,
            TaskState.MOVING_TO_TARGET: TaskStatus.MOVING,
            TaskState.RELEASING: TaskStatus.DELIVERING,
            TaskState.RETURNING: TaskStatus.MOVING,
            TaskState.ERROR: TaskStatus.ERROR,
        }

        msg.state = state_map.get(state, TaskStatus.IDLE)
        msg.progress = progress
        msg.message = message

        self.status_pub.publish(msg)

    def start_task(self):
        """开始任务"""
        self.get_logger().info('Starting pick and place task')
        self.current_state = TaskState.DETECTING
        self.publish_status(TaskState.DETECTING, 0.0, 'Searching for target')


def main(args=None):
    rclpy.init(args=args)
    node = TaskScheduler()

    # 自动开始任务
    node.start_task()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
