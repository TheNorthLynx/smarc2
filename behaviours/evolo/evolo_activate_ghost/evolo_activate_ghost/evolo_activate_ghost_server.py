import rclpy
import json
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from smarc_action_base.gentler_action_server import GentlerActionServer
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from evolo_msgs.msg import Topics as evoloTopics

class EvoloActivateGhost():

    def __init__(self,
                 node: Node,
                 action_name: str):
        self._node = node

        # Initialize the action server with the node and action name
        # Give it all the necessary callbacks
        self._as = GentlerActionServer(
            node,
            action_name,
            self._on_goal_received,
            self._on_cancel_received,
            self._prepare_loop,
            self._loop_inner,
            self._give_feedback,
            loop_frequency=2
        )

        # Initialize any necessary state for your specific action
        # These have nothing to do with the action server itself

        #Callback groups
        self.publisher_callback_group = ReentrantCallbackGroup()

        # Publishers
        self.ghost_pub = self._node.create_publisher(Twist, "/evolo/activate_ghost", 10)

        self._node.get_logger().info("Action server started")

    def _on_goal_received(self, goal_request: dict) -> bool:
        self._node.get_logger().info(f"Received goal request: {goal_request}")
        # Here you would typically validate the goal request
        # Return True to accept the goal, False to reject it
        goal_text = goal_request["json-params"]
        goal_dict = json.loads(goal_text)

        obstacle_radius = goal_dict["r"]
        time_to_collision = goal_dict["t"]
        obstacle_angle = goal_dict["a"]
        obstacle_speed = goal_dict["v"]

        msg = Twist()
        msg.linear.x = float(obstacle_radius)
        msg.linear.y = float(obstacle_speed)
        msg.linear.z = float(time_to_collision)
        msg.angular.x = float(obstacle_angle)
        self.ghost_pub.publish(msg)

        self._node.get_logger().info(f"Received: r = {obstacle_radius}, t = {time_to_collision}, a = {obstacle_angle}, v = {obstacle_speed}")
        return True
    
    def _on_cancel_received(self) -> bool:
        self._node.get_logger().info("Received cancel request")
        # Here you would typically handle the cancel request
        # Return True to accept the cancel, False to reject it
        return True

    def _prepare_loop(self) -> None:
        self._node.get_logger().info("Preparing loop for action execution")
        # Here you would typically set up any necessary state or resources
        # This is run once before the loop starts, after you accept the goal

    def _loop_inner(self) -> bool | None:
        # Return true right away. Hopefully one publication of "realease" is enough.
        # Otherwire keep publishing here for a few seconds
        return True

    def _give_feedback(self) -> str:
        feedback = "Ghost feedback"
        self._node.get_logger().info(feedback)
        # Here you would typically generate feedback for the action
        # This is run after each _loop_inner call
        return feedback

def main():
    rclpy.init()
    node = Node("evolo_activate_ghost_action_server")
    
    action_server = EvoloActivateGhost(node, "activate_ghost")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down evolo activate ghost acation server")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()