import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
import math
from my_msgs.msg import WaypointMsg

          

class Qr_recovery(Node):
    def __init__(self):
        super().__init__("qr_recovery") #Init node

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Puoi adattare il valore in base alle tue esigenze
        )

        cb_group_recovery = ReentrantCallbackGroup()
        cb_group_is_turtlebot_stopped = ReentrantCallbackGroup()
        cb_group_stop_recovery = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(String, '/recovery', self.listener_callback, qos_profile, callback_group=cb_group_recovery)
        self.sub_is_turtlebot_stopped = self.create_subscription(Bool, '/turtlebot_is_stopped', self.turtlebot_is_stopped_callback, qos_profile, callback_group=cb_group_is_turtlebot_stopped)
        self._sub_stop_recovery = self.create_subscription(Bool, "/stop_recovery", self.stop_recovery, qos_profile, callback_group=cb_group_stop_recovery)
        print("INIZIALIZZAZIONE")

        self.pub_end_wp = self.create_publisher(WaypointMsg, '/end_wp', qos_profile)

        self._turtlebot_is_stopped = False
        self._stop_recovery = False

        # self.finish_ack = self.create_publisher(Bool, "/finish_recovery", qos_profile) #Publisher



    def calculate_new_waypoint(self,x, y, yaw, distance):
        print(math.cos(math.radians(yaw)))
        new_x = x + distance * math.cos(math.radians(yaw))
        new_y = y + distance * math.sin(math.radians(yaw))
        return new_x, new_y

    def turtlebot_is_stopped_callback(self, msg):
        
        print("TURTLEBOT STOPPED", msg.data)
        self._turtlebot_is_stopped = msg.data

    def stop_recovery(self, msg):
        print("STOP RECOVERY:", msg.data)
        self._stop_recovery = msg.data

    def listener_callback(self, msg):
        print("CALLBACK")
        command = msg.data.split('_')
        self.get_logger().info(f'Received command: {msg.data}')

        if command[0] == 'Start':
            x = float(command[1])
            y = float(command[2])
            yaw = int(math.ceil(float(command[3])))

            if len(command) == 5:
                #caso 1
                alpha = int(math.ceil(float(command[4])))
                self.qr_code_seen( x, y, yaw,alpha)
            elif len(command) == 4:
                #caso 2
                self.pattern_movement(x, y, yaw)
            


    def qr_code_seen(self, x, y, yaw,alpha):
            # vado avanti di 5 cm
            print("QR code seen")
            new_x,new_y=self.calculate_new_waypoint(x, y, yaw+alpha, 1.0)
            end_wp_msg = WaypointMsg()
            end_wp_msg.x = new_x
            end_wp_msg.y = new_y
            end_wp_msg.direction = (yaw+alpha)%360
            if self._stop_recovery:
                print("Stop recovery")
                self._stop_recovery = False
                return
            
            

            self.pub_end_wp.publish(end_wp_msg)

            self._turtlebot_is_stopped = False
            
            print("New directions: ", new_x, new_y, end_wp_msg.direction)

            while not self._turtlebot_is_stopped:
                pass

            x = new_x
            y = new_y
            direction = (yaw+alpha)%360


            while True:
                new_x,new_y=self.calculate_new_waypoint(x, y, yaw+alpha, 1.0)
                end_wp_msg = WaypointMsg()
                end_wp_msg.x = new_x
                end_wp_msg.y = new_y
                end_wp_msg.direction = direction
                if self._stop_recovery:
                    print("Stop recovery")
                    self._stop_recovery = False
                    return
                
                

                self.pub_end_wp.publish(end_wp_msg)

                self._turtlebot_is_stopped = False
                
                print("New directions: ", new_x, new_y, end_wp_msg.direction)

                while not self._turtlebot_is_stopped:
                    pass

                x = new_x
                y = new_y



    def pattern_movement(self, x, y, yaw):
            while True:
                print("Pattern movement")
                end_wp_msg = WaypointMsg()
                end_wp_msg.x = x
                end_wp_msg.y = y
                
                # 20 gradi a sinistra
                end_wp_msg.direction = (yaw + 45)%360
                if self._stop_recovery:
                    print("Stop recovery")
                    self._stop_recovery = False
                    return
                print("Position - giro a sinistra: x: ", x, "y: ", y, "yaw: ", (yaw + 45)%360)

                

                self.pub_end_wp.publish(end_wp_msg)

                self._turtlebot_is_stopped = False

                while not self._turtlebot_is_stopped:
                    pass
                print("Ho girato a destra")
                # Torna dritto
                # rotation_pose = self.navigator.getPoseStamped([x, y], yaw)
                # self.navigator.startToPose(rotation_pose)
                # 20 gradi a destra
                end_wp_msg.direction = (yaw - 45)%360
                if self._stop_recovery:
                    print("Stop recovery")
                    self._stop_recovery = False
                    return
                print("Position - giro a destra: x: ", x, "y: ", y, "yaw: ", (yaw - 45)%360)

                

                self.pub_end_wp.publish(end_wp_msg)

                self._turtlebot_is_stopped = False

                while not self._turtlebot_is_stopped:
                    pass
                

                print("Ho girato a sinistra")
                # Torna dritto
                end_wp_msg.direction = yaw%360
                if self._stop_recovery:
                    print("Stop recovery")
                    self._stop_recovery = False
                    return
                print("Position - torno centrale: x: ", x, "y: ", y, "yaw: ", yaw%360)
                
                
                
                self.pub_end_wp.publish(end_wp_msg)

                self._turtlebot_is_stopped = False

                while not self._turtlebot_is_stopped:
                    pass
                print("Son tornato dritto")
                # Vai avanti di 5 cm
                new_x,new_y=self.calculate_new_waypoint(x, y, yaw, 1.0)
                end_wp_msg.x = new_x
                end_wp_msg.y = new_y
                if self._stop_recovery:
                    print("Stop recovery")
                    self._stop_recovery = False
                    return
                print("Position - vado avanti: x: ", x, "y: ", y, "yaw: ", yaw)

                

                self.pub_end_wp.publish(end_wp_msg)

                self._turtlebot_is_stopped = False

                print("Vado dritto")
                

                while not self._turtlebot_is_stopped:
                    pass
                
                x = new_x
                y = new_y
                print("New position: x: ", x, "y: ", y, "yaw: ", yaw)


def main():
    rclpy.init()
    recovery = Qr_recovery()
    executor = MultiThreadedExecutor()
    executor.add_node(recovery)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()