import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_pkg.utils.my_pose import MyPose
from nav_pkg.utils.junctions import Junctions
from nav_pkg.utils.constants import OffsetsRecovery


class Checkpoint(Node):

    def __init__(self):
        super().__init__('checkpoint')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Puoi adattare il valore in base alle tue esigenze
        )
        self.subscription = self.create_subscription(
            String,
            '/amcl_pose',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning
        self.current_pose = MyPose()
        self.curret_junction = None
        self.junctions = Junctions()
        self.in_junction = True

    def check_for_new_junction(self):
        new_junction = self.junctions.get_junction_by_point(self.current_pose.get_x(), self.current_pose.get_y())
        self.get_logger().info(f'I heard: {str(self.current_pose)}.\tCurrent junction = {new_junction}')
        if new_junction != self.curret_junction: 
            self.curret_junction = new_junction
            print(f'I\' IN A NEW JUNCTION: {self.curret_junction}') 

    def check_for_exit_from_junction(self): 
        if self.junctions.get_junction_by_point(self.current_pose.get_x(), self.current_pose.get_y(), OffsetsRecovery) is None: 
            # Se non è all'interno di un incrocio ma in un corridoio
            if self.in_junction == True: 
                # Se è appena uscito da un incrocio (ed è appena entrato in un corridoio)
                self.in_junction = False
                print(f'Just entered a corridor')


    def listener_callback(self, msg):
        self.current_pose.set_pose_from_msg(msg)
        self.check_for_new_junction()

        




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Checkpoint()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()