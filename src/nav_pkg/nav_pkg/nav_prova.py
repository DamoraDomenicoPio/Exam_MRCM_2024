import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.executors import SingleThreadedExecutor
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator, TaskResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_pkg.utils.waypoint import Waypoint
from my_msgs.msg import WaypointMsg

class Navigation(Node):
    def __init__(self):
        super().__init__("navigation_node") #Init node


        self.navigator = TurtleBot4Navigator()

        # Wait for Nav2
        self.navigator.waitUntilNav2Active()

        # Set initial pose
        initial_pose = self.navigator.getPoseStamped([-3.0, -10.0], TurtleBot4Directions.WEST)
        self.navigator.setInitialPose(initial_pose)


    def main(self):

        print("Starting navigation")



        self.navigator.startToPose(self.navigator.getPoseStamped([-3.0, -0.02], TurtleBot4Directions.SOUTH_WEST))
        print("Arrived at [-1.05, -0.02]")




def main():
    rclpy.init()
    navigation = Navigation()
    navigation.main()
    executor = SingleThreadedExecutor()
    executor.add_node(navigation)
    executor.spin()
