import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Puoi adattare il valore in base alle tue esigenze
        )
        self.publisher_ = self.create_publisher(String, '/amcl_pose', qos_profile)


    def publish_coordinates(self, x, y):
        msg = String()
        msg.data = f'{x},{y}'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    while True: 
        print('Press enter to choose the coordinates')
        input()
        x = float(input('x = '))
        y = float(input('y = '))
        minimal_publisher.publish_coordinates(x, y)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()