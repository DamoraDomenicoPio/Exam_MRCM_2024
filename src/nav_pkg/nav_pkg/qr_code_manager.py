import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
from std_msgs.msg import String, Bool
from dbr import *
from qreader import QReader
import cv2
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseWithCovarianceStamped

class QRCodeManager(Node):
    def __init__(self):
        super().__init__('qr_code_manager')
        # self.sub_process_image = self.create_subscription(CompressedImage, '/oakd/rgb/preview/image_raw/compressed', self.listener_callback, 10)
        cb_group_process_image = ReentrantCallbackGroup()
        cb_group_is_turtlebot_stopped = ReentrantCallbackGroup()
        pose_cb_group = ReentrantCallbackGroup()
        self.sub_process_image = self.create_subscription(String, '/prova_str', self.listener_callback, 10, callback_group=cb_group_process_image)
        self.sub_is_turtlebot_stopped = self.create_subscription(Bool, '/turtlebot_is_stopped', self.turtlebot_is_stopped_callback, 10, callback_group=cb_group_is_turtlebot_stopped)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10, callback_group=pose_cb_group)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Puoi adattare il valore in base alle tue esigenze
        )

        self.pub_navigation = self.create_publisher(String, '/road_sign', qos_profile)
        self.pub_recovery = self.create_publisher(String, '/recovery', qos_profile)
        self.bridge = CvBridge()
        self.old_sign = ""

        self._reader = QReader()

        self._turtlebot_is_stopped = False

        self.camera_fov = 60
        self._max_limit = 5 # Limit beyond which the robot is stationary
        self._i = 0 # Counter for the number of times the sign is detected as None
        self._is_recovery = False
        self._last_sign_is_None = False

        self._my_pose = self.MyPose()


    class MyPose():
        def __init__(self, x=0.0, y=0.0, yaw=0.0):
            self._x = x
            self._y = y
            self._yaw = yaw

        def set_pose(self, x=None, y=None, yaw=None):
            if x != None:
                self._x = x
            if y != None:
                self._y = y
            if yaw != None:
                self._yaw = yaw

        def get_x(self):
            return self._x
        
        def get_y(self):
            return self._y
        
        def get_yaw(self):
            return self._yaw

    def pose_callback(self, msg):
        
        current_pose = msg.pose.pose
        yaw = self.euler_from_quaternion(current_pose.orientation)
        self._my_pose.set_pose(current_pose.position.x, current_pose.position.y, math.degrees(yaw))
        print(f'Current pose: x={self._my_pose.get_x()}, y={self._my_pose.get_y()}, orientation={self._my_pose.get_yaw()}')
        # self.get_logger().info(f'Current pose: x={current_pose.position.x}, y={current_pose.position.y}, orientation={math.degrees(yaw)}')


    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        # roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        # pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return yaw





    def calculate_rotation_angle(self, bbox_center_x, img_width, camera_fov):
        offset_x = bbox_center_x - (img_width / 2)
        rotation_angle = (offset_x / (img_width / 2)) * (camera_fov / 2) #in gradi
        return rotation_angle

    

    def calculate_distance(self, fov, qr_code_width_pixels, qr_code_actual_width_cm, horizontal_resolution):
        """
        Calculate the distance from the QR code to the camera.
        
        Args:
        fov (float): Horizontal field of view angle of the camera in degrees.
        qr_code_width_pixels (int): Width of the QR code in pixels in the captured image.
        qr_code_actual_width_cm (float): Actual width of the QR code in centimeters.
        horizontal_resolution (int): Horizontal resolution of the camera in pixels.
        
        Returns:
        float: Distance from the QR code to the camera in centimeters.
        """
        # Calculate the field of view angle for the QR code in degrees
        qr_code_fov = (qr_code_width_pixels / horizontal_resolution) * fov
        
        # Convert the angle from degrees to radians for the tan function
        qr_code_fov_rad = math.radians(qr_code_fov)
        
        # Calculate the distance using the formula
        distance = qr_code_actual_width_cm / (2 * math.tan(qr_code_fov_rad / 2))
        
        return distance

    def _pub_msg(self, pub, str_msg):
        msg = String()
        msg.data = str_msg
        pub.publish(msg)


    def turtlebot_is_stopped_callback(self, data):
        print("Turtlebot is stopped", data.data)
        if data.data:
            self._turtlebot_is_stopped = True
        else:
            self._turtlebot_is_stopped = False

    # TODO: Implementare il filtro per i qr code
    def filter_qr_code(self, qr_code_list):
        pass

    def listener_callback(self, data): 

        # cv_image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        # read_signal_list = self._reader.detect_and_decode(image=cv_image)

        if data.data == "test":
            read_signal_list = []
        elif data.data == "None":
            read_signal_list = [None]
        else:
            read_signal_list = [data.data]
        
        if len(read_signal_list) > 0:
            # If something is detected we enter in this block

            # We take the detected signal
            if read_signal_list[0] == None:
                read_signal = "None"
            else:
                read_signal = read_signal_list[0]
            
            # If the detected signal is None we increment the counter
            if read_signal == "None":
                if self._is_recovery == False:
                    # The first None detected si send to navigator
                    if self._i == 0:
                        self._pub_msg(self.pub_navigation, read_signal)
                    
                    if self._i <= self._max_limit:
                        self._i += 1
                    elif self._turtlebot_is_stopped: # TODO: Prima c'era semplicemente un else. Adesso è stata aggiunta questa condizione perchè bisogna far partire la recovery solo se è fermo
                        # If the counter is greater than the limit we start the recovery
                        # From the moment that we see the signal, we know the qrcode position
                        self._pub_msg(self.pub_recovery, "Start_alpha") # TODO: Change the value of the message
                        self._i = 0
                        self._is_recovery = True
                else:
                    # If we are in recovery without seeing qr code and now we see the signal None, we restart the recovery,
                    # but, this time, we know the qrcode position.
                    # In this case, we don't send the None signal to the navigator, because we are already in recovery.
                    # TODO: vedere come l'ha gestito Ferdinando. Non so se mettere prima lo stop oppure fare direttamente start
                    if self._last_sign_is_None == False:
                        self._pub_msg(self.pub_recovery, "Start_alpha") # TODO: Change the value of the message
                        self._i = 0

                # Update last_sign_is_None variable
                self._last_sign_is_None = (read_signal == "None")

            # If the detected signal is not None we send it to the navigator and stop recovery
            else:
                if self._is_recovery == True:
                    self._pub_msg(self.pub_recovery, "Stop") # TODO: Assicurati che la recovery si sia fermata
                    self._is_recovery = False
                self._i = 0
                
                self._last_sign_is_None = False
                
                self._pub_msg(self.pub_navigation, read_signal)

        elif self._is_recovery == False:
            # If we don't detect anything and we are not in recovery we increment the counter
            # This means two things:
            # 1. The robot has reached the goal and has not detected any signal (goal_reached = True)
            # 2. The robot has detected a signal (last_sign = 'None'), but for some frames it has not detected anything
            if self._turtlebot_is_stopped == True or self._last_sign_is_None == True:
                self._i += 1
                if self._i <= self._max_limit:
                    # If the counter is greater than the limit we start the recovery
                    if self._last_sign_is_None == True:
                        self._pub_msg(self.pub_recovery, "Start_alpha") # TODO: Change the value of the message
                    else:
                        self._pub_msg(self.pub_recovery, "Start")
                    self._i = 0
                    self._is_recovery = True


            # if self._turtlebot_is_stopped == True or self._last_sign_is_None == True:
            #     self._i += 1
            #     if self._i == self._max_limit:
            #         # If the counter is greater than the limit we start the recovery

            #         if self._last_sign_is_None == True: # TODO: Assicurati che il turtlebot sia fermo
            #             self._pub_msg(self.pub_recovery, "Start_alpha") # TODO: Change the value of the message
            #         else:
            #             self._pub_msg(self.pub_recovery, "Start")
            #         self._i = 0
            #         self._is_recovery = True

        print("")
        print("Signal", read_signal_list)
        print("i", self._i)
        print("recovery", self._is_recovery)
        print("turtlebot_is_stopped", self._turtlebot_is_stopped)
        print("last_sign_is_None", self._last_sign_is_None)
            
        # Problemi che potrebbero sorgere:
        # 1. Quando la recovery blocca il task, il navigator potrebbe un attimo prima avviare un nuovo task. Questo potrebbe creare problemi. Credo.


        # if len(read_signal) > 0:
        #     msg = String()
        #     if read_signal[0] == None:
        #         msg.data = "None"
        #     else:
        #         msg.data = read_signal[0]
        #     self.pub_navigation.publish(msg)
        #     print(read_signal)

        
 
def main(args=None):
    rclpy.init(args=args)
    cv2.setUseOptimized(True)
    cv2.cuda.setDevice(0)
    qr_code_reader = QRCodeManager()
    rclpy.spin(qr_code_reader)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
