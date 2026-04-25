import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import socket
import json

class CommandBridge(Node):
    def __init__(self):
        super().__init__('pi_command_bridge')
        self.cmd_pub = self.create_publisher(TwistStamped, '/agriscout_base_controller/cmd_vel', 10)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind to all interfaces on the PC
        self.sock.bind(('0.0.0.0', 65432))
        self.sock.setblocking(False)
        
        self.timer = self.create_timer(0.05, self.listen_to_pi) 
        self.get_logger().info("PC Bridge Active. Listening to Pi...")

    def listen_to_pi(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            commands = json.loads(data.decode('utf-8'))
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.linear.x = float(commands['linear'])
            cmd.twist.angular.z = float(commands['angular'])
            
            self.cmd_pub.publish(cmd)
        except BlockingIOError:
            pass # No data waiting

def main():
    rclpy.init()
    node = CommandBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()