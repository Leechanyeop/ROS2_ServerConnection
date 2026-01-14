import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
import websockets
import json
import threading
import datetime


class CmdVelSubscriber(Node):
    def __init__(self, websocket_queue, loop):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.websocket_queue = websocket_queue
        self.loop = loop

    def listener_callback(self, msg):  # 서버로 전송하는 데이터 형식
        data = {
            "linear": {
                "x": msg.linear.x,
                "y": msg.linear.y,
                "z": msg.linear.z
            },
            "angular": {
                "x": msg.angular.x,
                "y": msg.angular.y,
                "z": msg.angular.z
            }
        }

        # 큐가 꽉 차 있다면(지연 발생 시), 가장 오래된 데이터를 버리고 최신 데이터만 넣음 (Drop Oldest)
        if self.websocket_queue.full():
            try:
                self.websocket_queue.get_nowait()
            except asyncio.QueueEmpty:
                pass

        # 스레드 안전하게 메인 루프의 큐에 데이터 넣기
        self.loop.call_soon_threadsafe(self.websocket_queue.put_nowait, json.dumps(data))


async def websocket_client(uri, queue):
    while True:
        try:
            async with websockets.connect(uri) as websocket:
                print(f"Connected to {uri}")

                """
                핸드 셰이크가 없을 경우 asp(서버)측에서 이게 어디서 온 정보인지 모름
                우리 구조상 웹소켓은 ros하나라 물론 그냥 해도 괜찮긴 함
                하지만 만약에 나중에 다른 정보를 또 웹소켓으로 보내고 싶을 경우 문제의 소지가 있음
                핸드 셰이크 안에 추가할 내용 
                """
                handshake = {
                    "type": "handshake",
                    "client_type": "ros2",
                }

                response = await websocket.recv()
                response_data = json.loads(response)  # return data를 json 으로 형변환

                if response_data.get('type') != 'handshake_ack' or not response_data.get('accepted'):
                    raise Exception('Handshake 오류')

                session_id = response_data['session_id']
                print(f'Handshake 성공. session:{session_id}')
                while True:
                    message = await queue.get()

                    payload = json.loads(message)
                    payload['type'] = 'cmd_vel'
                    payload['session_id'] = session_id
                    payload['timestamp'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                    await websocket.send(message)
                    # print(f"Sent: {message}") # 로그 제거로 속도 향상
        except Exception as e:
            print(f"Connection failed: {e}. Retrying in 5 seconds...")
            await asyncio.sleep(5)


def ros_spin(node):
    rclpy.spin(node)


async def main():
    # 윈도우 PC의 IP 주소를 입력하세요 (예: "ws://192.168.x.x:5000/ws")
    uri = "ws://192.168.10.30:5178/ws"

    # 지연 방지를 위해 큐 크기를 1로 제한 (항상 최신 1개만 유지)
    queue = asyncio.Queue(maxsize=1)

    loop = asyncio.get_running_loop()

    rclpy.init()
    node = CmdVelSubscriber(queue, loop)

    # Run ROS2 spin in a separate thread so it doesn't block asyncio
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        await websocket_client(uri, queue)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
