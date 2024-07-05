from nav_pkg.utils.constants import Directions, Signs, JunctionCoordinates
from nav_pkg.utils.junction_point import JunctionPoint
from nav_pkg.utils.junctions import Junctions
from nav_pkg.utils.waypoint import Waypoint
import numpy as np

import rclpy
from my_msgs.srv import GetNextWp
from rclpy.node import Node

class NextWaypointService(Node): 

    def __init__(self): 
        super().__init__('next_waypoint')
        self.server = self.create_service(GetNextWp, 'next_waypoint_service', self.next_waypoint_callback)
        self.junctions = Junctions()  # Oggetto che gestisce gli incroci

    def next_waypoint_callback(self, request, response):
        waypoint = self.junctions.get_next_junciton_waypoint(request.point_name, request.direction, request.sign)
        if not isinstance(waypoint, Waypoint): 
            raise TypeError('La funzione get_next_junction_waypoint deve restituire un waypoint')
        
        x = waypoint.get_x

        response.next_x = float(waypoint.get_x())
        response.next_y = float(waypoint.get_y())
        response.next_direction = int(waypoint.get_direction().value)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NextWaypointService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
