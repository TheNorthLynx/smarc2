#! /usr/bin/env python3

import math
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import TwistStamped
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
        self.evolo_pub = self.create_publisher(TwistStamped,
                                                f"{evoloTopics.EVOLO_SIM_CTRL_TO}", 1)
        self.logger.info(f"Sending ctrl messages to {evoloTopics.EVOLO_SIM_CTRL_TO}")

        # PD stuff
        self.p = 1.0
        self.d = 0.2
        self.dt = 0.5
        self.w_max = 30.0
        self.w_max_virtual = 7.0
        self.w_max_scale = self.w_max / self.w_max_virtual

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
        msg = TwistStamped()

        # Check termination
        if abs(self.yaw_setpoint - 4711.0) < 1:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
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

            # Apply the CBF
            w_safe, _ = self.calc_safe_u(w_des, -1)

            # Send the control
            msg.twist.linear.x = 8.0
            msg.twist.angular.z = w_safe * 180 / (self.w_max_scale * np.pi)
            self.logger.info(f"Sending lx = {msg.twist.linear.x}, az = {msg.twist.angular.z}")

        self.evolo_pub.publish(msg)


    def robot_odom_cb(self, msg : Odometry):
        self.robot_position = PoseStamped()
        self.robot_position.header = msg.header
        self.robot_position.pose = msg.pose.pose
        o_list = [self.robot_position.pose.orientation.x, self.robot_position.pose.orientation.y, self.robot_position.pose.orientation.z, self.robot_position.pose.orientation.w]
        _, _, z = euler_from_quaternion(o_list)
        self.robot_yaw = z
        # self.logger.info(f"Robot yaw updated: {self.robot_yaw}")

    def calc_safe_u(self, w_des, mu):
        """For a single fixed plane and fixed turning direction, calculate safe u"""
        opt_type = True
        x = [self.robot_position.pose.position.x, self.robot_position.pose.position.y, self.robot_yaw, 8.0]
        o = [40.0, 17.0, 0.0, 0.0]
        agent_radius = 1
        obstacle_radius = 3

        th = np.arctan2(x[1] - o[1], x[0] - o[0])

        # Calculate abbreviations
        u_max = self.w_max * np.pi / 180
        u_des = w_des * np.pi / 180
        r_min = x[3] / u_max
        v_h = np.cos(th) * o[2] + np.sin(th) * o[3]
        gamma = np.arcsin(v_h / x[3])
        beta = mu * (th - x[2]) - 0.5 * np.pi
        beta = ((beta + np.pi) % (2 * np.pi)) - np.pi

        # Calculate h, h_dot_c and h_dot_u
        h = np.cos(th) * (x[0] - o[0])
        h += np.sin(th) * (x[1] - o[1])
        h -= agent_radius + obstacle_radius
        # Check if plane is ok
        if h < 0:
            opt_type = False
        h -= r_min * (np.cos(gamma) - np.cos(beta))
        h -= r_min * beta * v_h / x[3]

        h_dot_c = np.cos(th) * (np.cos(x[2]) * x[3] - o[2])
        h_dot_c += np.sin(th) * (np.sin(x[2]) * x[3] - o[3])
        h_dot_u = mu * r_min * (np.sin(beta) + v_h / x[3])

        u = 0

        self.logger.info(f"h: {h}")

        # Check if plane is ok
        if h < 0:
            opt_type = False

        # If constraint not activated
        if h_dot_c + u_des * h_dot_u >= -self.alpha(h):
            u = u_des
            self.logger.info(f"Not active, h dot: {h_dot_c + u_des * h_dot_u}")

        # If constraint not possible
        elif h_dot_c + mu * u_max * h_dot_u < -self.alpha(h):
            u = mu * u_max
            opt_type = False
            self.logger.info(f"Not possible, h dot: {h_dot_c + mu * u_max * h_dot_u}")

        # If optimal u exists
        else:
            u = (-self.alpha(h) - h_dot_c) / h_dot_u
            self.logger.info(f"Optimal, h dot: {h_dot_c + u * h_dot_u}")

        return u, opt_type

    def update(self):
        pass

    def alpha(self, x):
        """A very simple alpha function"""
        return x


def main(args=None, namespace=None):
    rclpy.init(args=args)
    control_node = yaw_control()

    control_node.create_timer(1.0/control_node.update_rate, control_node.update)
    executor = MultiThreadedExecutor()
    executor.add_node(control_node)
    executor.spin()
