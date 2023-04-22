#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial

class PendularDraw(Node): # Instancia do nó que sera utilizado para fazer a comunicação 
    def __init__(self):
        super().__init__('pendular_draw') 
        self.previous_x_ = 0
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_subscriber_ = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.get_logger().info(f' Pendular Draw Node has been started')
        self.twist_msg_ = Twist()

    def pose_callback(self, pose: Pose):
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            self.twist_msg_.linear.x = 1.0
            self.twist_msg_.angular.z = 0.9
        else:
            self.twist_msg_.linear.x = 5.0
            self.twist_msg_.angular.z = 0.0
        self.publisher_.publish(self.twist_msg_)

        if pose.x > 5.5 and self.previous_x_ <= 5.5:
            self.previous_x_ = pose.x
            self.get_logger().info("Vemelho")
            self.call_set_pen_service(255, 0, 0, 3, 0)
        elif pose.x <= 5.5 and self.previous_x_ > 5.5:
            self.previous_x_ = pose.x
            self.get_logger().info("Branco")
            self.call_set_pen_service(255, 255, 255, 3, 0)
    
    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, 'turtle1/set_pen')
        while not client.wait_for_service(1.0):
            self.get_logger().info('service not available, waiting again...')
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
        
def main(args=None):
    rclpy.init(args=args)
    pendular_draw = PendularDraw()
    rclpy.spin(pendular_draw)
    rclpy.shutdown()

if __name__ == '__main__':
    main()