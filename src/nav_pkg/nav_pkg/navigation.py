import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rclpy.executors import MultiThreadedExecutor
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator, TaskResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_pkg.utils.waypoint import Waypoint
from my_msgs.msg import WaypointMsg
from my_msgs.srv import GetNextWp

class Navigation(Node):
    def __init__(self):
        super().__init__("navigation_node") #Init node
        

        # start_wp_cb_group = ReentrantCallbackGroup()
        end_wp_cb_group = ReentrantCallbackGroup()
        # goal_reached_cb_group = ReentrantCallbackGroup()
        road_sign_cb_group = ReentrantCallbackGroup()
        # turtlebot_is_stopped_cb_group = ReentrantCallbackGroup()

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
        self.pub_goal_reached = self.create_publisher(Bool, '/goal_reached', qos_profile)
        self.pub_turtlebot_is_stopped = self.create_publisher(Bool, '/turtlebot_is_stopped', qos_profile)
        self.pub_end_wp = self.create_publisher(WaypointMsg, '/end_wp', qos_profile)
        
        # Private attributes
        self._start_wp = None
        self._end_wp = None
        self._is_set_start_wp = True
        self._is_set_end_wp = False
        self._is_main_running = False
        self._navigator = TurtleBot4Navigator()

        self._navigator.waitUntilNav2Active()

        initial_pose = self._navigator.getPoseStamped([-3.0, -0.0], TurtleBot4Directions.EAST)
        # initial_pose = self._navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        
        self._navigator.setInitialPose(initial_pose)

        self.cli = self.create_client(GetNextWp, 'next_waypoint_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetNextWp.Request()

        # Timers
        # self.timer_check_wp = self.create_timer(1, self.check_waypoints)

    def send_request(self, point_name, direction, sign):
        self.req.point_name = point_name
        self.req.direction = direction
        self.req.sign = sign
        self.future = self.cli.call_async(self.req)
        print("Sending 1")
        self.future.add_done_callback(self.handle_service_response)
        print("Sending")

        # rclpy.spin_until_future_complete(self, self.future)
        # return self.future.result()

    def handle_service_response(self, future):
        response = future.result()
        self.get_logger().info(f"Service response received: {response}")
        # Process the response here
        end_wp_msg = WaypointMsg()
        end_wp_msg.x = 0.0
        end_wp_msg.y = 0.0
        end_wp_msg.direction = 0
        self.get_logger().info(f"Publishing end wp: {end_wp_msg}")
        self._end_main()
        self.pub_end_wp.publish(end_wp_msg)

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
            msg = Bool()
            msg.data = True
            self.pub_turtlebot_is_stopped.publish(msg)
        else:
            self._navigator.cancelTask()
            print("Next wp", self.send_request("G", 90, road_sign))

            # msg = WaypointMsg()
            # msg.x = 0.0
            # msg.y = 0.0
            # msg.direction = 0

            # print("End wp", msg.x, msg.y, msg.direction, msg)

            # self.pub_end_wp.publish(msg)

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
        print("End wp", msg)
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

        msg_turtlebot_is_stopped = Bool()
        msg_turtlebot_is_stopped.data = False
        self.pub_turtlebot_is_stopped.publish(msg_turtlebot_is_stopped)

        self._navigator.startToPose(goal_pose)

        print("Task is complete:", self._navigator.getResult(), self._navigator.getFeedback())

        self._navigator.info("Navigation completed.")

        msg = Bool()
        if self._navigator.getResult() == TaskResult.SUCCEEDED:
            msg.data = True
        else:
            msg.data = False

        self.pub_goal_reached.publish(msg)

        msg_turtlebot_is_stopped = Bool()
        msg_turtlebot_is_stopped.data = True
        self.pub_turtlebot_is_stopped.publish(msg_turtlebot_is_stopped)

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
