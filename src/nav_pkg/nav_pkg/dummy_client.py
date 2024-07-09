import sys

from my_msgs.srv import GetNextWp
import rclpy
from rclpy.node import Node
from nav_pkg.utils.junctions import Junctions


class DummyClient(Node):

    def __init__(self):
        super().__init__('dummy_client')
        self.cli = self.create_client(GetNextWp, 'next_waypoint_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetNextWp.Request()

    def send_request(self, point_name, direction, sign):
        self.req.point_name = point_name
        self.req.direction = direction
        self.req.sign = sign
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = DummyClient()
    
    response = minimal_client.send_request(str(sys.argv[1]), int(sys.argv[2]), str(sys.argv[3]))
    
    minimal_client.get_logger().info(
        'Result for get next wp: point_name %s direction %s sign %s \n= x: %s y: %s direction: %s' %
        (str(sys.argv[1]), str(sys.argv[2]), str(sys.argv[3]), response.next_x, response.next_y, response.next_direction))

    j = Junctions()
    name = j.get_junction_by_point(response.next_x, response.next_y)
    print(f'\nNext junction = {name}\n')
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()