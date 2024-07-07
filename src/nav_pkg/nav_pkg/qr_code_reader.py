import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
from std_msgs.msg import String
from dbr import *
from qreader import QReader
import cv2
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class QRCodeReader(Node):
    def __init__(self):
        super().__init__('qr_code_reader')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/oakd/rgb/preview/image_raw/compressed', 
            self.listener_callback,
            10)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Puoi adattare il valore in base alle tue esigenze
        )

        self.publisher_node = self.create_publisher(String, '/road_sign', qos_profile)
        self.bridge = CvBridge()
        self.old_sign = ""

        
        self._reader = QReader()

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

    def listener_callback(self, data): 
        # print("Received image")
        # try:

        #     cv_image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        #     readed_signal = self.decodeframe(cv_image)
        # except CvBridgeError as e:
        #     self.get_logger().error(f"Could not convert image: {e}")


        # print("Barcode Text :")

        
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        read_signal = self._reader.detect_and_decode(image=cv_image)
        
        if len(read_signal) > 0:
            msg = String()
            if read_signal[0] == None:
                msg.data = "None"
            else:
                msg.data = read_signal[0]
            self.publisher_node.publish(msg)
            print(read_signal)
        # msg = String()

        # try:
        #     cv_image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        # except CvBridgeError as e:
        #     self.get_logger().error(f"Could not convert image: {e}")
        #     return
        
        # try:
        #     road_sign_detected = str(decode(cv_image)[0].data.decode("utf-8"))
        #     msg.data = road_sign_detected
        #     self.publisher.publish(msg)
        #     road_sign = road_sign_detected

        # except IndexError:
        #     msg.data = "no_detection"
        #     self.publisher.publish(msg)
        #     road_sign= None
            
        # print("DETECTION: ",road_sign)
        
 
def main(args=None):
    rclpy.init(args=args)
    cv2.setUseOptimized(True)
    cv2.cuda.setDevice(0)
    qr_code_reader = QRCodeReader()
    rclpy.spin(qr_code_reader)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
