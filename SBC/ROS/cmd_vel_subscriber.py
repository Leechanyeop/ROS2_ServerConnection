import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import json
import threading

class CmdVelSocketSubscriber(Node):
    """
    C# 서버로부터 TCP 소켓을 통해 JSON 형식의 Cmd_vel 데이터를 수신하여
    ROS2의 /cmd_vel 토픽으로 관리(Publish)하는 노드입니다.
    """
    def __init__(self):
        super().__init__('cmd_vel_socket_subscriber')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('=== ROS2 Cmd_vel Socket Subscriber Node Started ===')
        
        # TCP 서버 설정 (C# CmdVelSender.cs에서 설정한 포트와 일치해야 함)
        self.host = '0.0.0.0'
        self.port = 9090 
        
        # 소켓 수신용 별도 스레드 실행 (ROS spin 방해 방지)
        self.socket_thread = threading.Thread(target=self.run_tcp_server, daemon=True)
        self.socket_thread.start()

    def run_tcp_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # 주소 재사용 설정 (서버 재시작 시 포트 점유 에러 방지)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen()
            self.get_logger().info(f'Socket Server listening on {self.host}:{self.port}')
            
            while rclpy.ok():
                try:
                    conn, addr = s.accept()
                    self.get_logger().info(f'Connected by {addr}')
                    
                    with conn:
                        buffer = ""
                        while rclpy.ok():
                            data = conn.recv(1024)
                            if not data:
                                break
                            
                            buffer += data.decode('utf-8')
                            
                            # C#에서 보낸 개행문자(\n)를 기준으로 메시지 파싱
                            if '\n' in buffer:
                                messages = buffer.split('\n')
                                for msg_str in messages[:-1]:
                                    if msg_str.strip():
                                        self.publish_twist(msg_str)
                                buffer = messages[-1]
                except Exception as e:
                    if rclpy.ok():
                        self.get_logger().error(f'Socket Error: {e}')

    def publish_twist(self, json_str):
        try:
            data = json.loads(json_str)
            
            twist = Twist()
            # C# CmdVelSender에서 보낸 JSON 구조에 맞춰 매핑
            twist.linear.x = float(data['linear']['x'])
            twist.linear.y = float(data['linear']['y'])
            twist.linear.z = float(data['linear']['z'])
            twist.angular.x = float(data['angular']['x'])
            twist.angular.y = float(data['angular']['y'])
            twist.angular.z = float(data['angular']['z'])
            
            self.publisher_.publish(twist)
            self.get_logger().info(f'Published to /cmd_vel: Linear.x={twist.linear.x}, Angular.z={twist.angular.z}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to parse or publish: {e} | Content: {json_str}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSocketSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
