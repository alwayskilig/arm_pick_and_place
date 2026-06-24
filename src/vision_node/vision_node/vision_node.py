import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # 声明参数
        self.declare_parameter('target_color', 'red')
        self.declare_parameter('min_area', 500)
        self.declare_parameter('max_area', 50000)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('camera_topic', '/camera/image')
        
        # 相机内参参数（需要根据实际相机标定）
        self.declare_parameter('camera_fx', 500.0)  # 焦距x
        self.declare_parameter('camera_fy', 500.0)  # 焦距y
        self.declare_parameter('camera_cx', 320.0)  # 光心x
        self.declare_parameter('camera_cy', 240.0)  # 光心y
        self.declare_parameter('camera_depth_scale', 0.001)  # 深度比例因子
        self.declare_parameter('default_depth', 0.5)  # 默认深度（无深度相机时）
        
        # 获取参数
        self.target_color = self.get_parameter('target_color').value
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.camera_frame = self.get_parameter('camera_frame').value
        camera_topic = self.get_parameter('camera_topic').value
        
        # 获取相机内参
        self.fx = self.get_parameter('camera_fx').value
        self.fy = self.get_parameter('camera_fy').value
        self.cx = self.get_parameter('camera_cx').value
        self.cy = self.get_parameter('camera_cy').value
        self.depth_scale = self.get_parameter('camera_depth_scale').value
        self.default_depth = self.get_parameter('default_depth').value
        
        # OpenCV桥接器
        self.bridge = CvBridge()
        
        # 订阅摄像头图像 (wpr_simulation2使用/camera/image)
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10)
        
        # 发布目标位姿
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/vision/target_pose',
            10)
        
        self.get_logger().info(f'Vision node initialized, subscribing to: {camera_topic}')
        self.get_logger().info(f'Camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}')
    
    def image_callback(self, msg):
        try:
            # 转换ROS图像到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 检测目标物体
            target_pose = self.detect_target(cv_image)
            
            if target_pose is not None:
                self.pose_pub.publish(target_pose)
                self.get_logger().info(f'Target detected at: x={target_pose.pose.position.x:.3f}, y={target_pose.pose.position.y:.3f}, z={target_pose.pose.position.z:.3f}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def detect_target(self, image):
        # 转换到HSV色彩空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 根据目标颜色设置阈值
        if self.target_color == 'red':
            lower_bound = np.array([0, 120, 70])
            upper_bound = np.array([10, 255, 255])
        elif self.target_color == 'green':
            lower_bound = np.array([36, 50, 70])
            upper_bound = np.array([89, 255, 255])
        elif self.target_color == 'blue':
            lower_bound = np.array([100, 150, 0])
            upper_bound = np.array([140, 255, 255])
        else:
            self.get_logger().warn(f'Unknown color: {self.target_color}')
            return None
        
        # 创建掩码
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # 形态学操作
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if self.min_area < area < self.max_area:
                # 计算轮廓中心
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    # 创建位姿消息
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = self.camera_frame
                    
                    # 使用相机内参计算3D坐标
                    # 像素坐标 -> 相机坐标系
                    # X_cam = (u - cx) * Z / fx
                    # Y_cam = (v - cy) * Z / fy
                    # Z_cam = depth (默认深度)
                    
                    z = self.default_depth  # 默认深度
                    x = (cx - self.cx) * z / self.fx
                    y = (cy - self.cy) * z / self.fy
                    
                    pose_msg.pose.position.x = x
                    pose_msg.pose.position.y = y
                    pose_msg.pose.position.z = z
                    
                    pose_msg.pose.orientation.w = 1.0
                    
                    return pose_msg
        
        return None


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()