import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

import socket
import time

SERVER_IP = '172.16.0.1'
SERVER_PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = (SERVER_IP, SERVER_PORT)
sock.bind(server_address)

class PosePublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'sa_pose/shoulder_marker', 10)
        self.publish_pose()

    def publish_pose(self):
        self.get_logger().info(f'Listening at {SERVER_IP}:{SERVER_PORT}')
        frame_id = 0
        while(True):
            data, address = sock.recvfrom(4096)
            received_str = data.decode('utf-8')
            # print(received_timestamp)
            # 以,分割，转为float
            data = [float(s) for s in received_str.split(',')]
            msg = PoseStamped()
            current_time = time.time()
            msg.header.stamp.sec = int(current_time)
            # msg.header.stamp.nanosec = int((current_time - int(current_time)) * 1e9)
            msg.pose.position.x = data[0]
            msg.pose.position.y = data[1]
            msg.pose.position.z = data[2]
            msg.pose.orientation.x = data[3]
            msg.pose.orientation.y = data[4]
            msg.pose.orientation.z = data[5]
            msg.pose.orientation.w = data[6]
            msg.header.frame_id = str(frame_id)
            
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: {msg.pose.orientation.x:.3} {msg.pose.orientation.y:.3} {msg.pose.orientation.z:.3} {msg.pose.orientation.w:.3}')
            
            frame_id += 1


def main(args=None):
    rclpy.init(args=args)

    pose_publisher = PosePublisher()

    rclpy.spin(pose_publisher)

    pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()