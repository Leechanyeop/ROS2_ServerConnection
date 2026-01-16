import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('ROS2 cmd_vel Publisher has been started.')

    def timer_callback(self):
        msg = Twist()
        # 원 형태의 움직임을 위해 단순한 값 설정
        msg.linear.x = 0.5   # 전진 속도
        msg.angular.z = 0.3  # 회전 속도
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: Linear={msg.linear.x}, Angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = CmdVelPublisher()
    try:
        rclpy.spin(cmd_vel_publisher)
    except KeyboardInterrupt:
        pass
    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
