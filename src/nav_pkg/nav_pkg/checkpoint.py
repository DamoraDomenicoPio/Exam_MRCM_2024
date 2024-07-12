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
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback_amcl,
            qos_profile,
            callback_group=amcl_cb_group)
        self.subscription  # prevent unused variable warning

        # Iscrizione a kidnapped
        kidnapped_cb_group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            Bool,
            '/kidnap_status',
            self.listener_callback_kidnapped,
            qos_profile,
            callback_group=kidnapped_cb_group)
        self.subscription  # prevent unused variable warning

        # Creazione del publisher
        self.publisher_ = self.create_publisher(WaypointMsg, '/end_wp', qos_profile)

        # Instance variables 
        self.current_pose = MyPose()
        self.curret_junction = sys.argv[1]
        assert self.curret_junction in ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'L'], 'Questo incrocio non esiste. Scegli fra A B C D E F G H I o L'
        self.junctions = Junctions()
        self.in_junction = True
        self.last_checkpoint = None
        self.last_direction = None

    # --------------------------------------------------------------------------------

    def yaw_to_direction(self, yaw): 
        direction = None
        if (yaw>=0 and yaw<=-45) or (yaw>=0 and yaw<=45): 
            direction = TurtleBot4Directions.NORTH
        elif yaw>=45 and yaw<=135: 
            direction = TurtleBot4Directions.WEST
        if (yaw>=135 and yaw<=-180) or (yaw>=-180 and yaw<=-135): 
            direction = TurtleBot4Directions.SOUTH
        elif yaw>=-180 and yaw<=-90: 
            direction = TurtleBot4Directions.EAST
        if direction is None: 
            raise Exception(f'La direzione {direction} non esiste')
        return direction

    def check_for_new_junction(self):
        new_junction = self.junctions.get_junction_by_point(self.current_pose.get_x(), self.current_pose.get_y())
        # self.get_logger().info(f'I heard: {str(self.current_pose)}.\tCurrent junction = {new_junction}')
        if new_junction != self.curret_junction: 
            old_junction_object = self.junctions.get_junction_by_name(self.curret_junction)
            direction = old_junction_object.get_direction_by_destination(new_junction)
            x, y = old_junction_object._get_bbox_point(direction)
            self.curret_junction = new_junction
            print(f'I\'m in a new junction: {self.curret_junction}. New checkpoint: x = {x}, y = {y}. Yaw = {self.current_pose.get_yaw()}') 
            self.last_checkpoint = [x, y]
            self.last_direction = direction



    def check_for_exit_from_junction(self): 
        if self.junctions.get_junction_by_point(self.current_pose.get_x(), self.current_pose.get_y(), OffsetsRecovery) is None: 
            # Se non è all'interno di un incrocio ma in un corridoio
            if self.in_junction == True: 
                # Se è appena uscito da un incrocio (ed è appena entrato in un corridoio)
                self.in_junction = False
                junction_object = self.junctions.get_junction_by_name(self.curret_junction)
                x = junction_object.get_x()
                y = junction_object.get_y()
                print(f'Just entered a corridor. New checkpoint: x = {x}, y = {y}. Yaw = {self.current_pose.get_yaw()}')
                self.last_checkpoint = [x, y]
                self.last_direction = self.yaw_to_direction(self.current_pose.get_yaw()).value
        else:
            self.in_junction = True

                

    # Publishing and subscriber callbacks ----------------------------------------------

    def publish_checkpoint(self):
        end_wp_msg = WaypointMsg()
        end_wp_msg.x = self.last_checkpoint[0]
        end_wp_msg.y = self.last_checkpoint[1]
        end_wp_msg.direction = self.last_direction
        print(f'\nPublished x = {self.last_checkpoint[0]}, y = {self.last_checkpoint[1]}, direction = {self.last_direction}')

        # msg = String()
        # msg.data = f'{self.last_checkpoint[0]},{self.last_checkpoint[1]},{self.last_direction.value}'
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)


    def listener_callback_amcl(self, msg):
        self.current_pose.set_pose_from_msg(msg)
        self.check_for_new_junction()
        self.check_for_exit_from_junction()

    def listener_callback_kidnapped(self, msg): 
        print('Kidnapped')
        if msg.data:
            self.publish_checkpoint()


        




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