import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rclpy.executors import MultiThreadedExecutor
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator, TaskResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_pkg.utils.waypoint import Waypoint
from my_msgs.msg import WaypointMsg

class Navigation(Node):
    def __init__(self):
        super().__init__("navigation_node") #Init node
        

        # start_wp_cb_group = ReentrantCallbackGroup()
        end_wp_cb_group = ReentrantCallbackGroup()
        goal_reached_cb_group = ReentrantCallbackGroup()
        road_sign_cb_group = ReentrantCallbackGroup()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Puoi adattare il valore in base alle tue esigenze
        )

        # Subscribers
        # # self.sub_start_wp = self.create_subscription(WaypointMsg, "/start_wp", self.start_wp_callback, qos_profile, callback_group=start_wp_cb_group)
        self.sub_end_wp = self.create_subscription(WaypointMsg, "/end_wp", self.end_wp_callback, qos_profile, callback_group=end_wp_cb_group)
        self.sub_road_sign = self.create_subscription(String, "/road_sign", self.road_sign_callback , qos_profile, callback_group=road_sign_cb_group)
        # Publishers
        self.pub_goal_reached = self.create_publisher(Bool, '/goal_reached', 10, callback_group=goal_reached_cb_group)
        
        # Private attributes
        self._start_wp = None
        self._end_wp = None
        self._is_set_start_wp = True
        self._is_set_end_wp = False
        self._is_main_running = False
        self._navigator = TurtleBot4Navigator()

        self._navigator.waitUntilNav2Active()

        initial_pose = self._navigator.getPoseStamped([5.0, -0.02], TurtleBot4Directions.SOUTH)
        
        self._navigator.setInitialPose(initial_pose)


        
        self._threashold_check_x = 20
        self._threashold_check_y = 20
        self._threashold_check_direction = 30

        # Timers
        # self.timer_check_wp = self.create_timer(1, self.check_waypoints)



    def _int_to_direction(self, num):
        if num == 0:
            return TurtleBot4Directions.NORTH
        elif num == 270:
            return TurtleBot4Directions.EAST
        elif num == 180:
            return TurtleBot4Directions.SOUTH
        elif num == 90:
            return TurtleBot4Directions.WEST


    def road_sign_callback(self, msg):
        road_sign = msg.data
        print("Road sign detected:", road_sign)
        if road_sign == 'None':
            self._navigator.cancelTask()
        else:
            self._navigator.cancelTask()
            # Calculate next_wp
            # publish end_wp

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

        if self._is_set_end_wp and not self._is_main_running:
            self.navigation()
        

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

        if self._is_set_start_wp and not self._is_main_running:
            self.navigation()
            

    def _end_main(self):
        self._is_main_running = False
        self._start_wp = None
        self._end_wp = None
        self._is_set_end_wp = False


    def navigation(self):
        print("Inizio navigation")

        goal_pose = self._navigator.getPoseStamped([self._end_wp.get_x(), self._end_wp.get_y()], self._end_wp.get_direction())
        self._navigator.startToPose(goal_pose)
        self._navigator.cancelTask()

        print("Task is complete:", self._navigator.getResult(), self._navigator.getFeedback())

        self._navigator.info("Navigation completed.")

        msg = Bool()
        if self._navigator.getResult() == TaskResult.SUCCEEDED:
            msg.data = True
        else:
            msg.data = False

        self.pub_goal_reached.publish(msg)
        self._end_main()
        print("Fine main")


        


def main():
    rclpy.init()

    navigation = Navigation()
    executor = MultiThreadedExecutor()
    param = rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True) #Creating 'use_sim_time' node parameter
    navigation.set_parameters([param]) 
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
