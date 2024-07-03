import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
from std_msgs.msg import String
from dbr import *
from qreader import QReader

class QRCodeReader(Node):
    def __init__(self):
        super().__init__('qr_code_reader')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/oakd/rgb/preview/image_raw/compressed', 
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, '/road_sign', 10)
        self.bridge = CvBridge()

        
        self._reader = QReader()



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

        if len(read_signal) > 0  and read_signal[0] is not None:
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
    qr_code_reader = QRCodeReader()
    rclpy.spin(qr_code_reader)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
