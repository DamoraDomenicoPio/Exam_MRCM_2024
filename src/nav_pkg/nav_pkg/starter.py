import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_pkg.utils.constants import Signs
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
        self.publisher_ = self.create_publisher(String, '/road_sign', qos_profile)


    def publish_straighton(self, sign=Signs.STRAIGHTON):
        if sign == '': 
            sign = Signs.STRAIGHTON.value
        assert sign in [Signs.GOBACK.value, Signs.LEFT.value, Signs.RIGHT.value, Signs.STOP.value, Signs.STRAIGHTON.value], 'Questo segnale non esiste'
        msg = String()
        msg.data = sign
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    print('Test object created')

    while True: 
        print('Press enter to send a STRAIGHTON, write the desidered signal otherwise:')
        sign = input('Signal = ')
        print(f'Sign = {sign}')
        minimal_publisher.publish_straighton(sign)


if __name__ == '__main__':
    main()