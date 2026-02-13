#! /usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion

from evolo_msgs.msg import Topics as evoloTopics
from smarc_msgs.msg import Topics as smarcTopics
from smarc_control_msgs.msg import Topics as ControlTopics
import json

# import controller as ctrl

# Basic PID regulator
class yaw_control(Node):
    def __init__(self):
        super().__init__("yaw_control")
        self.logger = self.get_logger()
        self.logger.info("Yaw control initiated!")

        self.declare_node_parameters()

        self.update_rate = float(self.get_parameter("update_rate").value)
        self.logger.info(f"update rate: {self.update_rate}")
        self.logger.info("Hello fromt the simulated controller!")
        self.robot_name = self.get_parameter("robot_name").value

        self.yaw_setpoint = None
        self.yaw_setpoint_time = None

        # Odometry sub
        self.robot_position = PoseStamped()
        self.robot_yaw = None
        self.robot_sub = self.create_subscription(Odometry, smarcTopics.ODOM_TOPIC, self.robot_odom_cb, 10)

        # Control inputs.
        self.ctrl_sub = self.create_subscription(Float32, 
                                 f"{ControlTopics.CONTROL_YAW_TOPIC}", self.yaw_cb, 1)
        # Outputs
        self.evolo_pub = self.create_publisher(Twist,
                                                f"{evoloTopics.EVOLO_SIM_CTRL_TO}", 1)
        self.logger.info(f"Sending ctrl messages to {evoloTopics.EVOLO_SIM_CTRL_TO}")

        # PD stuff
        self.p = 1.0
        self.d = 0.2
        self.dt = 0.5
        self.w_max = 13.0

        self.prev_yaw_error = 0.0

    def time_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def declare_node_parameters(self):
        self.declare_parameter("update_rate", 1)
        self.declare_parameter("robot_name", "evolo")

    def yaw_cb(self, msg):
        """Callback when reciving a new desired yaw"""
        self.yaw_setpoint = msg.data
        self.yaw_setpoint_time = self.time_now()
        # self.logger.info(f"Recived target yaw: {self.yaw_setpoint}")
        # self.logger.info(f"Current yaw: {self.robot_yaw}")
        msg = Twist()

        # Check termination
        if abs(self.yaw_setpoint - 4711.0) < 1:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.logger.info("Finished!")
        # Get desired yaw rate using a PD-controller
        elif self.robot_yaw is not None:
            yaw_error = self.yaw_setpoint - self.robot_yaw
            # self.logger.info(f"Yaw error: {yaw_error}")

            w_des = (
                self.p * yaw_error
                + self.d * (yaw_error - self.prev_yaw_error) / self.dt
            )
            # self.logger.info(f"w_des (rad): {w_des}")
            # Transfer to degrees in Unity coordiantes
            w_des = w_des * 180.0 / 3.14
            
            if w_des > self.w_max:
                w_des = self.w_max
            elif w_des < -self.w_max:
                w_des = -self.w_max
            # self.logger.info(f"w_des (deg): {w_des}")

            self.prev_error_prev = yaw_error

            # Send the control
            msg.linear.x = 8.0
            msg.angular.z = w_des

        self.evolo_pub.publish(msg)
        self.logger.info(f"Sending lx = {msg.linear.x}, az = {msg.angular.z}")


    def robot_odom_cb(self, msg : Odometry):
        self.robot_position = PoseStamped()
        self.robot_position.header = msg.header
        self.robot_position.pose = msg.pose.pose
        o_list = [self.robot_position.pose.orientation.x, self.robot_position.pose.orientation.y, self.robot_position.pose.orientation.z, self.robot_position.pose.orientation.w]
        x, y, z = euler_from_quaternion(o_list)
        self.robot_yaw = z
        # self.logger.info(f"Robot yaw updated: {self.robot_yaw}")

    def update(self):
        pass
        # now = self.time_now()

        # msg = Twist()
        # if self.yaw_setpoint_time is not None and now-self.yaw_setpoint_time < 1 and self.yaw_setpoint is not None:
        #     #Convert yaw to NED and degrees
        #     target_course = -math.degrees(self.yaw_setpoint) + 90
        #     while(target_course < 0):
        #         target_course+=360
        #     while(target_course >= 360):
        #         target_course -= 360

        #     #TODO send YAW command to evolo
        #     msg = String()

        #     target = {"ctt": target_course,"dtt": 100, "sogAim": "fly"}
        #     msg.data = json.dumps({"setTarget": target})
        #     self.evolo_pub.publish(msg)


        #     self.logger.info(f"sending target course={target_course}")

        # # else:
        
        # if self.yaw_setpoint is not None:
        #     msg.linear.x = 8.0
        #     msg.angular.z = self.yaw_setpoint
        # else:
        #     msg.linear.x = 0.0
        #     msg.angular.z = 0.0
        # self.evolo_pub.publish(msg)
        # self.logger.info(f"Sending lx = {msg.linear.x}, az = {msg.angular.z}")


def main(args=None, namespace=None):
    rclpy.init(args=args)
    control_node = yaw_control()

    control_node.create_timer(1.0/control_node.update_rate, control_node.update)
    executor = MultiThreadedExecutor()
    executor.add_node(control_node)
    executor.spin()
