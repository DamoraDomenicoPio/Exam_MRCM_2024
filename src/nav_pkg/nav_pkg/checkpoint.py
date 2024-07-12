import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_pkg.utils.my_pose import MyPose
from nav_pkg.utils.junctions import Junctions
from nav_pkg.utils.constants import OffsetsRecovery
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped
from my_msgs.msg import WaypointMsg
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions
import sys
from irobot_create_msgs.msg import KidnapStatus


class Checkpoint(Node):

    def __init__(self):
        super().__init__('checkpoint')

        # Iscrizione a amcl_pose
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Puoi adattare il valore in base alle tue esigenze
        )
        amcl_cb_group = ReentrantCallbackGroup()
        # self.subscription = self.create_subscription(
        #     PoseWithCovarianceStamped,
        #     '/amcl_pose',
        #     self.listener_callback_amcl,
        #     qos_profile,
        #     callback_group=amcl_cb_group)
        self.subscription = self.create_subscription(
            String,
            '/amcl_pose',
            self.listener_callback_amcl,
            qos_profile,
            callback_group=amcl_cb_group)
        self.subscription  # prevent unused variable warning

        # Iscrizione a kidnapped
        qos_profile_kidnapped = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Puoi adattare il valore in base alle tue esigenze
        )
        kidnapped_cb_group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            KidnapStatus,
            '/kidnap_status',
            self.listener_callback_kidnapped,
            qos_profile_kidnapped,
            callback_group=kidnapped_cb_group)
        self.subscription  # prevent unused variable warning

        # Creazione del publisher
        self.publisher_ = self.create_publisher(WaypointMsg, '/start_wp', qos_profile)

        # Instance variables 
        self.current_pose = MyPose()
        self.curret_junction = 'C' # sys.argv[1]
        # assert self.curret_junction in ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'L'], 'Questo incrocio non esiste. Scegli fra A B C D E F G H I o L'
        self.junctions = Junctions()
        self.in_junction = True
        self.last_checkpoint = [0,0]
        self.last_direction = 0
        self.not_initialized = True
        self.kidnapped_published = False
        self.print_checkpoint()

    # --------------------------------------------------------------------------------

    def old_yaw_to_direction(self, yaw): 
        direction = None
        if (yaw<=0 and yaw>=-45) or (yaw>=0 and yaw<=45): 
            direction = TurtleBot4Directions.NORTH
        elif yaw>=45 and yaw<=135: 
            direction = TurtleBot4Directions.WEST
        if (yaw>=135 and yaw<=180) or (yaw>=-180 and yaw<=-135): 
            direction = TurtleBot4Directions.SOUTH
        elif yaw>=-180 and yaw<=-90: 
            direction = TurtleBot4Directions.EAST
        if direction is None: 
            raise Exception(f'La direzione {direction} (yaw = {yaw}) non esiste')
        return direction
    
    def yaw_to_direction(self, yaw):
        yaw = yaw % 360
        direction = None
        if (yaw>=0 and yaw<=45) or (yaw>=315 and yaw<=360): 
            direction = TurtleBot4Directions.NORTH
        elif yaw>=45 and yaw<=135: 
            direction = TurtleBot4Directions.WEST
        elif yaw>=135 and yaw<=225: 
            direction = TurtleBot4Directions.SOUTH
        elif yaw>=225 and yaw<=315: 
            direction = TurtleBot4Directions.EAST
        else: 
            raise Exception(f'La direzione {direction} (yaw = {yaw}) non esiste')
        return direction
    
    def initialize_checkpoint(self):
        if self.not_initialized: 
            self.last_direction = self.yaw_to_direction(self.current_pose.get_yaw()).value
            self.last_checkpoint = [self.current_pose.get_x(), self.current_pose.get_y()]
            self.not_initialized = False
            print('Checkpoint initialized')
            self.print_checkpoint()

    def check_for_new_junction(self):
        new_junction = self.junctions.get_junction_by_point(self.current_pose.get_x(), self.current_pose.get_y())
        # self.get_logger().info(f'I heard: {str(self.current_pose)}.\tCurrent junction = {new_junction}')
        if new_junction != self.curret_junction: 
            old_junction_object = self.junctions.get_junction_by_name(self.curret_junction)
            direction = old_junction_object.get_direction_by_destination(new_junction)
            x, y = old_junction_object._get_bbox_point(direction)
            self.curret_junction = new_junction
            print(f'\nMid corridor - Entering the {self.curret_junction} zone.') 
            self.last_checkpoint = [x, y]
            self.last_direction = direction
            self.print_checkpoint()



    def check_for_exit_from_junction(self): 
        if self.junctions.get_junction_by_point(self.current_pose.get_x(), self.current_pose.get_y(), OffsetsRecovery) is None: 
            # Se non è all'interno di un incrocio ma in un corridoio
            if self.in_junction == True: 
                # Se è appena uscito da un incrocio (ed è appena entrato in un corridoio)
                self.in_junction = False
                junction_object = self.junctions.get_junction_by_name(self.curret_junction)
                x = junction_object.get_x()
                y = junction_object.get_y()
                print(f'\nJust entered a corridor and exited from the {self.curret_junction} junction')
                self.last_checkpoint = [x, y]
                self.last_direction = self.yaw_to_direction(self.current_pose.get_yaw()).value
                self.print_checkpoint()
        else:
            self.in_junction = True

    def print_checkpoint(self): 
        print(f'New checkpoint: x = {self.last_checkpoint[0]}, y = {self.last_checkpoint[1]}, direction = {self.junctions.convert_directions(self.last_direction)}')

                

    # Publishing and subscriber callbacks ----------------------------------------------

    def publish_checkpoint(self):
        end_wp_msg = WaypointMsg()
        end_wp_msg.x = self.last_checkpoint[0]
        end_wp_msg.y = self.last_checkpoint[1]
        end_wp_msg.direction = self.last_direction
        print('Published')
        self.print_checkpoint()
        self.publisher_.publish(end_wp_msg)


    def listener_callback_amcl(self, msg):
        self.current_pose.set_pose_from_msg(msg)
        self.initialize_checkpoint()
        self.check_for_new_junction()
        self.check_for_exit_from_junction()

    def listener_callback_kidnapped(self, msg): 
        if msg.is_kidnapped:
            if self.kidnapped_published: 
                print(f'Kidnapped {str(msg.is_kidnapped)}')
                self.publish_checkpoint()
                self.kidnapped_published = True
        else: 
            self.kidnapped_published = False

        




def main(args=None):
    rclpy.init(args=args)

    checkpoint = Checkpoint()

    executor = MultiThreadedExecutor()
    executor.add_node(checkpoint)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        checkpoint.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()