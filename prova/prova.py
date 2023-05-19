#! /usr/bin/env python3 

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from turtlesim.msg import Pose

from queue import Queue, LifoQueue

class MoveTurtle(Node): 
    def __init__(self, verify_timer = 0.1):
        super.__init__('move_turtle')

        self.publisher_ = self.create_publisher(
            msg_type=Twist,
            topic='cmd_vel',
            qos_profile=10
        )

        self.subscriber_ = self.create_subscription(
            msg_type= Pose,
            topic='pose',
            callback=self.pose_callback,
            qos_profile=10
        )

        self.timer = self.create_timer(
            verify_timer,
            self.update_velocity_callback
        )

        self.twist_msg_ = Twist() 
        
        self.position_queue_ = Queue() 
        
        self.position_stack_ = LifoQueue()

        self.pointer = None

    def pose_callback(self, points):
        # Não consegui desenvolver a lógica, desculpa Nicola!
        pass

    def update_velocity_callback():
        # Não consegui desenvolver a lógica, desculpa Nicola!
        pass


def main(args=None):
    rclpy.init(args=args)
    
    points = [[0.0, 0,5], [0.5, 0.0], [0.0, 0.5], [0.0, 1.0], [1.0, 0.0]]

    move_turtle = MoveTurtle()

    move_turtle.pose_callback(points)

    rclpy.spin(move_turtle)

    move_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


