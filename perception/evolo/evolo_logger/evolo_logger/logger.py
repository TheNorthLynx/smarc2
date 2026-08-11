#! /usr/bin/env python3

import math
import rclpy
import time
import numpy as np
from rclpy.node import Node
from rclpy.time import Duration, Time
from std_msgs.msg import String, Float32
from geometry_msgs.msg import TwistStamped
from rosgraph_msgs.msg import Clock
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PointStamped
from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped

from evolo_msgs.msg import Topics as EvoloTopics
from smarc_msgs.msg import Topics as smarcTopics
from smarc_control_msgs.msg import Topics as ControlTopics
import json


class logger(Node):
    """Sends virtual obstacles on collision course with Evolo."""
    def __init__(self):
        super().__init__("logger")
        self.logger = self.get_logger()
        self.logger.info("Logging initiated!")

        self.declare_node_parameters()

        self.update_rate = float(self.get_parameter("update_rate").value)
        self.logger.info(f"update rate: {self.update_rate}")
        self.robot_name = self.get_parameter("robot_name").value

        # Keep track of Evolo using TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True
        )

        self.total_log_time = float(self.get_parameter("log_time").value)
        self.logger.info(f"Logging for: {self.total_log_time} s")
        self.t = 0

        # Make sure TF2 is ready
        # tf_not_ready = True
        # while tf_not_ready:
        #     try:
        #         t = self.tf_buffer.lookup_transform(
        #             target_frame="evolo/odom",
        #             source_frame="base_footprint",
        #             time=Time(seconds=0),
        #             timeout=Duration(seconds=1),
        #         )
        #         tf_not_ready = False
        #     except Exception as e:
        #         self.get_logger().error(f"Transform failed: {e}")
        #         time.sleep(3.0)

        self.t_list = np.linspace(0, self.total_log_time, int(self.total_log_time + 1))
        self.a_list = np.zeros((int(self.total_log_time + 1), 2))
        self.o_list = np.zeros((int(self.total_log_time + 1), 2))

        # Start obstacle subscriber
        self.o = [0, 0]
        self.obstacle_sub = self.create_subscription(Odometry, f"{EvoloTopics.EVOLO_CBF_OBSTACLES}", self.obstacle_cb, 10)

        self.a = [0, 0]
        self.agent_sub = self.create_subscription(Odometry, f"/footprint_odom", self.agent_cb, 10)

        time.sleep(2.0)

        self.logger.info(f"Logging starts now!")

    def clock_cb(self, msg):
        self.current_time = msg.clock

    def declare_node_parameters(self):
        self.declare_parameter("update_rate", 1)
        self.declare_parameter("robot_name", "")
        self.declare_parameter("log_name", "")
        self.declare_parameter("log_time", 1.0)

    def update(self):
        """Calculate the updated position of the obstacle in odom frame, then send it"""
        if self.t <= self.total_log_time:
            # tf_evolo = self.tf_buffer.lookup_transform(
            #     target_frame="base_footprint",
            #     source_frame="evolo/odom",
            #     time=Time(seconds=0),
            #     timeout=Duration(seconds=1),
            # )
            # self.a_list[self.t, 0] = tf_evolo.transform.translation.x
            # self.a_list[self.t, 1] = tf_evolo.transform.translation.y
            self.a_list[self.t, 0] = self.a[0]
            self.a_list[self.t, 1] = self.a[1]
            self.o_list[self.t, 0] = self.o[0]
            self.o_list[self.t, 1] = self.o[1]
        elif self.t == self.total_log_time + 1:
            log_name = self.get_parameter("log_name").value
            self.logger.info(f"Saving as {log_name}.npy")
            with open(f"{log_name}.npy", "wb") as f:
                np.save(f, self.t_list)
                np.save(f, self.a_list)
                np.save(f, self.o_list)
            self.logger.info(f"Logging done, saved.")
        self.t += 1

    def obstacle_cb(self, msg):
        self.o[0] = msg.pose.pose.position.x
        self.o[1] = msg.pose.pose.position.y

    def agent_cb(self, msg):
        self.a[0] = msg.pose.pose.position.x
        self.a[1] = msg.pose.pose.position.y


def main(args=None, namespace=None):
    rclpy.init(args=args)
    ghost_node = logger()

    ghost_node.create_timer(1.0/ghost_node.update_rate, ghost_node.update)
    executor = MultiThreadedExecutor()
    executor.add_node(ghost_node)
    executor.spin()
