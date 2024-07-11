import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_pkg.utils.my_pose import MyPose
from nav_pkg.utils.junctions import Junctions
from nav_pkg.utils.constants import OffsetsRecovery
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


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
            String,
            '/amcl_pose',
            self.listener_callback_amcl,
            qos_profile)
        self.subscription  # prevent unused variable warning

        # Iscrizione a kidnapped
        kidnapped_cb_group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            String,
            '/kidnapped',
            self.listener_callback_kidnapped,
            qos_profile)
        self.subscription  # prevent unused variable warning

        # Creazione del publisher
        self.publisher_ = self.create_publisher(String, '/end_wp', qos_profile)

        # Instance variables 
        self.current_pose = MyPose()
        self.curret_junction = 'C'
        self.junctions = Junctions()
        self.in_junction = True
        self.last_checkpoint = None
        self.last_direction = None

    def check_for_new_junction(self):
        new_junction = self.junctions.get_junction_by_point(self.current_pose.get_x(), self.current_pose.get_y())
        self.get_logger().info(f'I heard: {str(self.current_pose)}.\tCurrent junction = {new_junction}')
        if new_junction != self.curret_junction: 
            old_junction_object = self.junctions.get_junction_by_name(self.curret_junction)
            direction = old_junction_object.get_direction_by_destination(new_junction)
            self.last_direction = direction
            x, y = old_junction_object._get_bbox_point(direction)
            self.curret_junction = new_junction
            print(f'I\'m in a new junction: {self.curret_junction}. New checkpoint: x = {x}, y = {y}') 
            self.last_checkpoint = [x, y]



    def check_for_exit_from_junction(self): 
        if self.junctions.get_junction_by_point(self.current_pose.get_x(), self.current_pose.get_y(), OffsetsRecovery) is None: 
            # Se non è all'interno di un incrocio ma in un corridoio
            if self.in_junction == True: 
                # Se è appena uscito da un incrocio (ed è appena entrato in un corridoio)
                self.in_junction = False
                junction_object = self.junctions.get_junction_by_name(self.curret_junction)
                x = junction_object.get_x()
                y = junction_object.get_y()
                print(f'Just entered a corridor. New checkpoint: x = {x}, y = {y}')
                self.last_checkpoint = [x, y]

    def publish_checkpoint(self):
        msg = String()
        msg.data = f'{self.last_checkpoint[0]},{self.last_checkpoint[1]},{self.last_direction.value}'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


    def listener_callback_amcl(self, msg):
        self.current_pose.set_pose_from_msg(msg)
        self.check_for_new_junction()
        self.check_for_exit_from_junction()

    def listener_callback_kidnapped(self, msg): 
        self.publish_checkpoint()


        




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Checkpoint()
    executor = MultiThreadedExecutor()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()