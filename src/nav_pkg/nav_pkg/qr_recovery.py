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
        cb_group_stop_recovery_case_1 = ReentrantCallbackGroup()
        cb_group_stop_recovery_case_2 = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(String, '/recovery', self.listener_callback, qos_profile, callback_group=cb_group_recovery)

        self.sub_is_turtlebot_stopped = self.create_subscription(Bool, '/turtlebot_is_stopped', self.turtlebot_is_stopped_callback, qos_profile, callback_group=cb_group_is_turtlebot_stopped)
        self._sub_stop_recovery_case_1 = self.create_subscription(Bool, "/stop_recovery_case_1", self.stop_recovery_case_1, qos_profile, callback_group=cb_group_stop_recovery_case_1)
        self._sub_stop_recovery_case_2 = self.create_subscription(Bool, "/stop_recovery_case_2", self.stop_recovery_case_2, qos_profile, callback_group=cb_group_stop_recovery_case_2)
        print("INIZIALIZZAZIONE")

        self.pub_end_wp = self.create_publisher(WaypointMsg, '/end_wp', qos_profile)

        self._turtlebot_is_stopped = False
        self._stop_recovery_case_1 = True
        self._stop_recovery_case_2 = True

        # self.finish_ack = self.create_publisher(Bool, "/finish_recovery", qos_profile) #Publisher



    def calculate_new_waypoint(self,x, y, yaw, distance):
        print(math.cos(math.radians(yaw)))
        new_x = x + distance * math.cos(math.radians(yaw))
        new_y = y + distance * math.sin(math.radians(yaw))
        return new_x, new_y

    def turtlebot_is_stopped_callback(self, msg):
        
        print("TURTLEBOT STOPPED", msg.data)
        self._turtlebot_is_stopped = msg.data

    def stop_recovery_case_1(self, msg):
        print("STOP RECOVERY CASE 1:", msg.data)
        self._stop_recovery_case_1 = msg.data

    def stop_recovery_case_2(self, msg):
        print("STOP RECOVERY CASE 2:", msg.data)
        self._stop_recovery_case_2 = msg.data

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
                self._stop_recovery_case_1 = False
                alpha = int(math.ceil(float(command[4])))
                self.qr_code_seen( x, y, yaw,alpha)
            elif len(command) == 4:
                #caso 2
                self._stop_recovery_case_2 = False
                self.pattern_movement(x, y, yaw)
            
    def _int_to_int_direction(self, num):
        if (num >= 0 and num <= 22.5) or (num >= 337.5 and num <= 360):
            return 0
        elif num >= 247.5 and num <= 292.5:
            return 270
        elif num >= 157.5 and num <= 202.5:
            return 180
        elif num >= 67.5 and num <= 112.5:
            return 90
        elif num >= 22.5 and num <= 67.5:
            return 45
        elif num >= 112.5 and num <= 157.5:
            return 135
        elif num >= 202.5 and num <= 247.5:
            return 225
        elif num >= 292.5 and num <= 337.5:
            return 315
        else:
            print("Error in _int_to_int_direction")

    def qr_code_seen(self, x, y, yaw,alpha):
            while not self._stop_recovery_case_2:
                print("Attendo stop recovery case 2")
                pass
            # vado avanti di 5 cm
            print("QR code seen")
            new_x,new_y=self.calculate_new_waypoint(x, y, yaw+alpha, 0.5)
            end_wp_msg = WaypointMsg()
            end_wp_msg.x = new_x
            end_wp_msg.y = new_y

            direction = self._int_to_int_direction((yaw+alpha)%360)
            end_wp_msg.direction = direction
            if self._stop_recovery_case_1:
                print("Stop recovery case 1")
                
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

                new_x,new_y=self.calculate_new_waypoint(x, y, yaw+alpha, 0.5)
                end_wp_msg = WaypointMsg()
                end_wp_msg.x = new_x
                end_wp_msg.y = new_y
                end_wp_msg.direction = direction
                if self._stop_recovery_case_1:
                    print("Stop recovery case 1")
                    return
                
                

                self.pub_end_wp.publish(end_wp_msg)

                self._turtlebot_is_stopped = False
                
                print("New directions: ", new_x, new_y, end_wp_msg.direction)

                while not self._turtlebot_is_stopped:
                    pass

                x = new_x
                y = new_y



    def pattern_movement(self, x, y, yaw):
            yaw = self._int_to_int_direction(yaw%360)

            print("Old x and y", x, y, yaw)

            if (yaw >= 0 and yaw <= 45) or (yaw >= 315 and yaw <= 360):
                print("A")
                x += 0.5
            elif yaw >= 135 and yaw <= 225:
                print("B")
                x -= 0.5
            if yaw >= 45 and yaw <= 135:
                print("C")
                y += 0.5
            elif yaw >= 225 and yaw <= 315:
                print("D")
                y -= 0.5

            print("New x and y", x, y)


            while True:
                print("Pattern movement")
                end_wp_msg = WaypointMsg()
                end_wp_msg.x = x
                end_wp_msg.y = y
                
                # 20 gradi a sinistra
                end_wp_msg.direction = (yaw + 45)%360
                if self._stop_recovery_case_2:
                    print("Stop recovery case 2")
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
                if self._stop_recovery_case_2:
                    print("Stop recovery case 2")
                    return
                print("Position - giro a destra: x: ", x, "y: ", y, "yaw: ", (yaw - 45)%360)

                

                self.pub_end_wp.publish(end_wp_msg)

                self._turtlebot_is_stopped = False

                while not self._turtlebot_is_stopped:
                    pass
                

                print("Ho girato a sinistra")
                # Torna dritto
                end_wp_msg.direction = yaw%360
                if self._stop_recovery_case_2:
                    print("Stop recovery case 2")
                    return
                print("Position - torno centrale: x: ", x, "y: ", y, "yaw: ", yaw%360)
                
                
                
                self.pub_end_wp.publish(end_wp_msg)

                self._turtlebot_is_stopped = False

                while not self._turtlebot_is_stopped:
                    pass
                print("Son tornato dritto")
                # Vai avanti di 5 cm
                new_x,new_y=self.calculate_new_waypoint(x, y, yaw, 0.5)
                end_wp_msg.x = new_x
                end_wp_msg.y = new_y
                if self._stop_recovery_case_2:
                    print("Stop recovery case 2")
                    return
                print("Position - vado avanti: x: ", new_x, "y: ", new_y, "yaw: ", yaw)

                

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