import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator, TaskResult
import time
from tf2_ros import TransformListener
from tf2_ros import TransformStamped
from tf2_geometry_msgs import TransformStamped as TF2TransformStamped
import math
import argparse
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from utils.waypoint import Waypoint


class Navigation(Node):
    def __init__(self):
        super().__init__("navigation_node") #Init node

        start_wp_cb_group = ReentrantCallbackGroup()
        end_wp_cb_group = ReentrantCallbackGroup()
        pose_cb_group = ReentrantCallbackGroup()
        goal_reached_cb_group = ReentrantCallbackGroup()
        stop_main_cb_group = ReentrantCallbackGroup()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Puoi adattare il valore in base alle tue esigenze
        )

        # Subscribers
        self.sub_start_wp = self.create_subscription(WaypointMsg, "/start_wp", self.start_wp_callback, qos_profile, callback_group=start_wp_cb_group)
        self.sub_end_wp = self.create_subscription(WaypointMsg, "/end_wp", self.end_wp_callback, qos_profile, callback_group=end_wp_cb_group)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10, callback_group=pose_cb_group)
        self.sub_stop_main = self.create_subscription(Bool, '/stop_main', self.stop_main_callback, 10, callback_group=stop_main_cb_group)

        # Publishers
        self.pub_goal_reached = self.create_publisher(Bool, '/goal_reached', 10, callback_group=goal_reached_cb_group)

        # Private attributes
        self._start_wp = None
        self._end_wp = None
        self._is_set_start_wp = False
        self._is_set_end_wp = False
        self._is_main_running = False
        self._my_pose = self.MyPose()
        self._navigator = TurtleBot4Navigator()
        self._navigator.waitUntilNav2Active()
        
        self._threashold_check_x = 20
        self._threashold_check_y = 20
        self._threashold_check_direction = 30

        # Timers
        self.timer_check_wp = self.create_timer(1, self.check_waypoints)

    class MyPose():
        def __init__(self, x=0.0, y=0.0, yaw=0.0):
            self._x = x
            self._y = y
            self._yaw = yaw

        def set_pose(self, x=None, y=None, yaw=None):
            if x != None:
                self._x = x
            if y != None:
                self._y = y
            if yaw != None:
                self._yaw = yaw

        def get_x(self):
            return self._x
        
        def get_y(self):
            return self._y
        
        def get_yaw(self):
            return self._yaw


    def _int_to_direction(self, num):
        if num == 0:
            return TurtleBot4Directions.NORTH
        elif num == 270:
            return TurtleBot4Directions.EAST
        elif num == 180:
            return TurtleBot4Directions.SOUTH
        elif num == 90:
            return TurtleBot4Directions.WEST

    def pose_callback(self, msg):
        
        current_pose = msg.pose.pose
        _, _, yaw = self.euler_from_quaternion(current_pose.orientation)
        self._my_pose.set_pose(current_pose.position.x, current_pose.position.y, math.degrees(yaw))
        # self.get_logger().info(f'Current pose: x={current_pose.position.x}, y={current_pose.position.y}, orientation={math.degrees(yaw)}')

    def stop_main_callback(self, msg):
        if self._is_main_running:
            if msg.data:
                try:
                    raise Exception("Stop main!")
                except Exception as e:
                    self.get_logger().error(f"Exception in stop_main_callback: {e}")

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return roll, pitch, yaw

    def start_wp_callback(self, msg):
        if not self._is_set_start_wp and not self._is_main_running:
            x = msg.x
            y = msg.y
            
            direction = self._int_to_direction(msg.direction)
            initial_pose = self._navigator.getPoseStamped([x, y], direction)
            self._navigator.setInitialPose(initial_pose)
            self._is_set_start_wp = True
            print("Setting start wp:", x, y, direction)


        else:
            print("Start wp is already set")

        msg_true = Bool()
        msg_true.data = True
        self.pub_ack_start_wp.publish(msg_true)
        

    def end_wp_callback(self, msg):
        if not self._is_set_end_wp and not self._is_main_running:
            
            x = msg.x
            y = msg.y
            
            direction = self._int_to_direction(msg.direction)
            self._end_wp = Waypoint(x, y, direction)
            self._is_set_end_wp = True
            print("Setting end wp:", x, y, direction)

        else:
            print("End wp is already set")

        msg_true = Bool()
        msg_true.data = True
        self.pub_ack_end_wp.publish(msg_true)
        

    def check_waypoints(self):
        if self._is_set_start_wp and self._is_set_end_wp and not self._is_main_running:
            print("check_waypoints", self._is_main_running)

            self._is_main_running = True
            self.main()
        else:
            print("Non inizio:", self._is_set_start_wp, self._is_set_end_wp, self._is_main_running)
            

    def _end_main(self):
        self._is_main_running = False
        self._start_wp = None
        self._end_wp = None
        self._is_set_end_wp = False

    def _check_end_wp(self):
        if abs(self._my_pose.get_x() - self._end_wp.get_x()) > self._threashold_check_x:
            return False
        if abs(self._my_pose.get_y() - self._end_wp.get_y()) > self._threashold_check_y:
            return False
        yaw = self._my_pose.get_yaw()
        if yaw < 0:
            yaw += 360
        if abs(yaw - self._end_wp.get_direction()) > self._threashold_check_direction:
            return False
        return True


    def main(self):
        try:
            print("Inizio main")

            # Set initial pose

            # Undock
            # navigator.undock()
            
            # Navigate to end waypoint
            goal_pose = self._navigator.getPoseStamped([self._end_wp.get_x(), self._end_wp.get_y()], self._end_wp.get_direction())
            self._navigator.startToPose(goal_pose)

            print("Task is complete:", self._navigator.getResult(), self._navigator.getFeedback())
            



            self._navigator.info("Navigation completed.")

            




            if not self._check_end_wp():
                print("non hai raggiunto il punto finale")

            msg = Bool()
            if self._navigator.getResult() == TaskResult.SUCCEEDED and self._check_end_wp():
                msg.data = True
            else:
                msg.data = False

        except Exception as e:
            x = self._my_pose.get_x()
            y = self._my_pose.get_y()
            yaw = int(self._my_pose.get_yaw())
            if yaw < 0:
                yaw += 360

            print("Fermato in:", x, y, yaw)
            # goal_pose = self._navigator.getPoseStamped([x, y], yaw)
            # self._navigator.startToPose(goal_pose)
            msg = Bool()
            msg.data = False
        finally:

            print("My pose:", self._my_pose.get_x(), self._my_pose.get_y(), self._my_pose.get_yaw())
            self.pub_goal_reached.publish(msg)
            self._end_main()
            print("Fine main")


        


def main():
    rclpy.init()




    navigation = Navigation()
    executor = MultiThreadedExecutor()
    # param = rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True) #Creating 'use_sim_time' node parameter
    # navigation.set_parameters([param]) 
    executor.add_node(navigation)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        navigation.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
