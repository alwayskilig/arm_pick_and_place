import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from arm_pick_and_place.msg import TaskCommand
import time


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        self.declare_parameter('open_position', 0.04)
        self.declare_parameter('close_position', 0.0)
        self.declare_parameter('close_force', 20.0)
        self.declare_parameter('action_timeout', 5.0)

        self.open_position = self.get_parameter('open_position').value
        self.close_position = self.get_parameter('close_position').value
        self.close_force = self.get_parameter('close_force').value
        self.action_timeout = self.get_parameter('action_timeout').value

        self.current_position = self.open_position
        self.is_open = True

        self.command_sub = self.create_subscription(
            TaskCommand,
            '/task/command',
            self.command_callback,
            10)

        self.state_pub = self.create_publisher(
            Float64,
            '/gripper/state',
            10)

        # 发布到仿真环境的夹爪控制
        self.mani_ctrl_pub = self.create_publisher(
            JointState,
            '/wpb_home/mani_ctrl',
            10)

        self.timer = self.create_timer(0.1, self.publish_state)

        self.get_logger().info('Gripper controller initialized')

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.command_type}')

        if msg.command_type == TaskCommand.GRASP:
            self.close()
        elif msg.command_type == TaskCommand.DELIVER:
            self.open()
        elif msg.command_type == TaskCommand.RESET:
            self.open()

    def close(self):
        self.get_logger().info('Closing gripper')

        start_time = time.time()
        while time.time() - start_time < self.action_timeout:
            if self.current_position > self.close_position:
                self.current_position -= 0.001
                self.publish_state()
                self.publish_to_sim()
                time.sleep(0.01)
            else:
                self.current_position = self.close_position
                self.is_open = False
                self.get_logger().info('Gripper closed')
                break

    def open(self):
        self.get_logger().info('Opening gripper')

        start_time = time.time()
        while time.time() - start_time < self.action_timeout:
            if self.current_position < self.open_position:
                self.current_position += 0.001
                self.publish_state()
                self.publish_to_sim()
                time.sleep(0.01)
            else:
                self.current_position = self.open_position
                self.is_open = True
                self.get_logger().info('Gripper opened')
                break

    def publish_state(self):
        msg = Float64()
        msg.data = self.current_position

        self.state_pub.publish(msg)

    def publish_to_sim(self):
        """发布到仿真环境"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['gripper']
        # 转换夹爪位置到仿真接口 (0.0 ~ 0.05m)
        gripper_pos = self.current_position / 2.0  # 缩放到仿真范围
        msg.position = [gripper_pos]
        self.mani_ctrl_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()