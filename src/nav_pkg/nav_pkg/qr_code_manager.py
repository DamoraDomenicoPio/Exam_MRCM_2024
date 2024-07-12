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
from nav_pkg.utils.junctions import Junctions
import argparse

class QRCodeManager(Node):
    def __init__(self, start_junction):
        super().__init__('qr_code_manager')
        
        cb_group_process_image = ReentrantCallbackGroup()
        cb_group_is_turtlebot_stopped = ReentrantCallbackGroup()
        pose_cb_group = ReentrantCallbackGroup()
        cb_group_process_image = ReentrantCallbackGroup()
        # self.sub_process_image = self.create_subscription(String, '/prova_str', self.listener_callback, 10, callback_group=cb_group_process_image)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Puoi adattare il valore in base alle tue esigenze
        )

        self.sub_process_image = self.create_subscription(CompressedImage, '/oakd/rgb/preview/image_raw/compressed', self.listener_callback, 10, callback_group = cb_group_process_image)
        self.sub_is_turtlebot_stopped = self.create_subscription(Bool, '/turtlebot_is_stopped', self.turtlebot_is_stopped_callback, qos_profile, callback_group=cb_group_is_turtlebot_stopped)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, qos_profile, callback_group=pose_cb_group)
        


        self.pub_navigation = self.create_publisher(String, '/road_sign', qos_profile)
        self.pub_recovery = self.create_publisher(String, '/recovery', qos_profile)
        self.pub_stop_recovery_case_1 = self.create_publisher(Bool, '/stop_recovery_case_1', qos_profile)
        self.pub_stop_recovery_case_2 = self.create_publisher(Bool, '/stop_recovery_case_2', qos_profile)
        self._msg_true = Bool()
        self._msg_true.data = True

        self.bridge = CvBridge()
        self.old_sign = ""

        self._reader = QReader()

        self._turtlebot_is_stopped = False

        self._camera_fov = 69
        self._qr_code_actual_width_cm = 10
        self._img_width = 1000
        self._img_height = 1000

        self._max_limit = 25 # Limit beyond which the robot is stationary
        self._i = 0 # Counter for the number of times the sign is detected as None
        self._is_recovery = False
        self._last_sign_is_None = False
        
        
        # For evoiding the first None signal to be sent to the navigator (False positive)
        self._counter_None = 0
        self._None_qr_code_width = 10 # TODO: test this value

        self._old_junction = start_junction

        self._junctions = Junctions()

        self._my_pose = self.MyPose()

        self._last_position_qr = ""

        # This variable is used to avoid start counnting the None signal if we see it for the first time
        self._start_conter_None = False

        self._stop_manager = False


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

        print("Nella funzione che calcola la distanca:", qr_code_fov, qr_code_fov_rad, math.tan(qr_code_fov_rad / 2), qr_code_actual_width_cm / (2 * math.tan(qr_code_fov_rad / 2)))
        
        # Calculate the distance using the formula
        distance = qr_code_actual_width_cm / (2 * math.tan(qr_code_fov_rad / 2))

        distance = distance/100
        
        return abs(distance)

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

    def get_qr_informations(self, cv_image):
        '''Return the distance and the rotation angle of the qr code
        
        Args:
        cv_image (np.array): Image in which the qr code is detected
        
        Returns:
        float: Rotation angle of the qr code
        float: Distance from the qr code to the camera
        '''

        detected = self._reader.detect(image=cv_image)
        start_point = (int(detected[0]['bbox_xyxy'][0]), int(detected[0]['bbox_xyxy'][1]))
        end_point = (int(detected[0]['bbox_xyxy'][2]), int(detected[0]['bbox_xyxy'][3]))
        bbox_center_x = (start_point[0] + end_point[0]) / 2
        # bbox_center_y = (start_point[1] + end_point[1]) / 2
        rotation_angle = -self.calculate_rotation_angle(bbox_center_x, self._img_width, self._camera_fov)
        print("Angolo di rotazione: ", rotation_angle)
        qr_code_size = 0

        qr_code_width = end_point[0] - start_point[0]
        qr_code_height = end_point[1] - start_point[1]
        
        if qr_code_width > qr_code_height:
            qr_code_size = qr_code_width
        else:
            qr_code_size = qr_code_height
        qr_code_distance = self.calculate_distance(self._camera_fov, qr_code_size, self._qr_code_actual_width_cm, self._img_width)
        print("Distanza: ", qr_code_distance)
        return rotation_angle, qr_code_distance, qr_code_width, qr_code_height

    def get_qr_junction(self, distance, rotation_angle):
        '''Return the junction in which the qr code is located
        
        Args:
        distance (float): Distance from the qr code to the camera
        rotation_angle (float): Rotation angle of the qr code
        
        Returns:
        str: Junction in which the qr code is located
        '''

        print("Calcolo x:", math.cos(math.radians(self._my_pose.get_yaw() + rotation_angle)), distance, self._my_pose.get_x())
        print("Calcolo y:", math.sin(math.radians(self._my_pose.get_yaw() + rotation_angle)), distance, self._my_pose.get_y())
        print("Estimated qr position:")
        x_estimated = self._my_pose.get_x() + distance * math.cos(math.radians(self._my_pose.get_yaw() + rotation_angle))
        y_estimated = self._my_pose.get_y() + distance * math.sin(math.radians(self._my_pose.get_yaw() + rotation_angle))
        print("x: ", x_estimated)
        print("y: ", y_estimated)
        return self._junctions.get_junction_by_point(x_estimated, y_estimated)
        
    def filter_image(self, distance, rotation_angle, qr_code_string):
        '''Return True if the qr is the same as the previous one, False otherwise
        
        Args:
        cv_image (np.array): Image in which the qr code is detected
        qr_code_string (str): String of the detected qr code
        
        Returns:
        bool: True if the qr is the same as the previous one, False otherwise
        '''

        new_junc = self.get_qr_junction(distance, rotation_angle)
        print("Nuovo incrocio:", new_junc, self._old_junction)
        if new_junc == self._old_junction:
            return True
        else:
            if qr_code_string != "None":
                print("Update old junction in", new_junc)
                self._old_junction = new_junc
            return False
        
    def _update_counter_None(self, qr_str, qr_code_width):
        if qr_str == "":
            self._counter_None -= 1
            self._counter_None = max(0, self._counter_None)
        elif qr_str != "None":
            self._counter_None = 0
            self._start_conter_None = False
        else:
            if qr_code_width > self._None_qr_code_width:
                self._counter_None += 1
                if self._counter_None > 5:
                    self._counter_None = 5
            else:
                print("Qr troppo piccolo", qr_code_width)
            if self._counter_None == 3:
                self._start_conter_None = True
            # else:
            #     self._counter_None = 0

    def listener_callback(self, data): 

        cv_image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        # print("Image size", cv_image.shape)
        read_signal_list = self._reader.detect_and_decode(image=cv_image)

        if not self._stop_manager:

            # if data.data == "":
            #     read_signal_list = []
            # elif data.data == "None":
            #     read_signal_list = [None]
            # else:
            #     read_signal_list = [data.data]

            read_signal = ""
            qr_code_width = None
            
            if len(read_signal_list) > 0:
                # If something is detected we enter in this block
                
                
                # We take the detected signal
                if read_signal_list[0] == None:
                    read_signal = "None"
                else:
                    read_signal = read_signal_list[0]
                    self._last_position_qr = ""
                    if read_signal == "STOP":
                        self._stop_manager = True

                        return

                rotation_angle, distance, qr_code_width, _ = self.get_qr_informations(cv_image)
                # rotation_angle, distance, qr_code_width = 72, 15, 30

                # If the detected signal is not the same as the previous one we send it to the navigator
                if not self.filter_image(distance, rotation_angle, read_signal):
                    
                    # If the detected signal is None we increment the counter
                    if read_signal == "None":
                        self._last_position_qr = str(round(self._my_pose.get_x(), 2)) + "_" + str(round(self._my_pose.get_y(), 2)) + "_" + str(round(self._my_pose.get_yaw(), 2)) + "_" + str(int(rotation_angle))
                        if self._is_recovery == False:
                            # The first None detected si send to navigator
                            print("Inviiato il None?", self._counter_None, self._i, not self._turtlebot_is_stopped)
                            if self._counter_None == 5 or self._i > self._max_limit - 5:
                                if not self._turtlebot_is_stopped:
                                    self._pub_msg(self.pub_navigation, read_signal)
                                    print("Inviiato il None")
                            
                            if self._i <= self._max_limit:
                                if self._start_conter_None:
                                    self._i += 1
                                    print("i + 1", self._i)
                            elif self._turtlebot_is_stopped: # TODO: Prima c'era semplicemente un else. Adesso è stata aggiunta questa condizione perchè bisogna far partire la recovery solo se è fermo
                                # If the counter is greater than the limit we start the recovery
                                # From the moment that we see the signal, we know the qrcode position
                                
                                str_msg = "Start_" + self._last_position_qr 
                                self._pub_msg(self.pub_recovery, str_msg)
                                print("Inizio recovery:", str_msg)
                                self._i = 0
                                self._is_recovery = True
                        else:
                            # If we are in recovery without seeing qr code and now we see the signal None, we restart the recovery,
                            # but, this time, we know the qrcode position.
                            # In this case, we don't send the None signal to the navigator, because we are already in recovery.
                            # TODO: vedere come l'ha gestito Ferdinando. Non so se mettere prima lo stop oppure fare direttamente start
                            if self._last_sign_is_None == False:
                                self.pub_stop_recovery_case_2.publish(self._msg_true)
                                str_msg = "Start_" + self._last_position_qr
                                self._pub_msg(self.pub_recovery, str_msg)
                                self._i = 0
                                self._last_sign_is_None = True


                        # Update last_sign_is_None variable
                        if self._start_conter_None:
                            self._last_sign_is_None = True

                    # If the detected signal is not None we send it to the navigator and stop recovery
                    else:
                        if self._is_recovery == True:
                            # self._pub_msg(self.pub_recovery, "Stop") # TODO: Assicurati che la recovery si sia fermata
                            self.pub_stop_recovery_case_1.publish(self._msg_true)
                            self.pub_stop_recovery_case_2.publish(self._msg_true)
                            self._is_recovery = False
                        print("Signal", read_signal)
                        self._i = 0
                        
                        self._last_sign_is_None = False
                        
                        self._pub_msg(self.pub_navigation, read_signal)
                else:
                    read_signal = ""
            elif self._is_recovery == False:
                # If we don't detect anything and we are not in recovery we increment the counter
                # This means two things:
                # 1. The robot has reached the goal and has not detected any signal (goal_reached = True)
                # 2. The robot has detected a signal (last_sign = 'None'), but for some frames it has not detected anything
                if self._turtlebot_is_stopped == True or self._last_sign_is_None == True:

                    self._i += 1
                    print("i + 1", self._i, self._turtlebot_is_stopped, self._last_sign_is_None)
                    if self._i > self._max_limit:

                        ################################################################################### # TODO: In questo pezzo di codice si tiene conto del fatto che il qrcode visto precedentemente fosse un None, e adesso non si vede più nulla. Credo che questo venga in ognicaso gestito automaticamente
                        # If the counter is greater than the limit we start the recovery
                        
                        if self._last_sign_is_None == True:
                            self._pub_msg(self.pub_recovery, "Start_" + self._last_position_qr) # TODO: Change the value of the message
                        else:
                            self._pub_msg(self.pub_recovery, "Start_" + str(round(self._my_pose.get_x(), 2)) + "_" + str(round(self._my_pose.get_y(), 2)) + "_" + str(round(self._my_pose.get_yaw(), 2)) )
                        ###################################################################################





                        self._i = 0
                        self._is_recovery = True

            self._update_counter_None(read_signal, qr_code_width)

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
            print("counter_None", self._counter_None)
            print("start_counter_None", self._start_conter_None)

        else:
            print("I read STOP. Finished")
            self.pub_stop_recovery_case_1.publish(self._msg_true)
            self.pub_stop_recovery_case_2.publish(self._msg_true)
            
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
    
    parser = argparse.ArgumentParser(description='QR Code Manager')
    parser.add_argument('--start_junction', type=str, help='Junction where the robot starts')

    start_junction = parser.parse_args().start_junction
    rclpy.init(args=args)
    cv2.setUseOptimized(True)
    cv2.cuda.setDevice(0)
    qr_code_reader = QRCodeManager(start_junction)
    rclpy.spin(qr_code_reader)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
