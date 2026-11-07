import rclpy
import cv2
from rclpy.node import Node
from std_msgs.msg import String

# Détection de marqueurs ArUco avec OpenCV
dico = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dico, params)

# URL de la video via DroidCam. Pour résoudre le pb de l'accès à la caméra via WSL
URL = "http://192.168.0.20:4747/video?640x480"
# noeud ROS
class Points_img(Node) :
    def __init__ (self) : 
        self.cap = cv2.VideoCapture(URL)
        super().__init__('points_img')
        self.publisher = self.create_publisher(String, 'position_arUco_img', 10)
        timer = 0.01
        self.timer = self.create_timer(timer, self.timer_callback)

    def timer_callback(self) :
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Image non reçue')
            return
        corners, ids, rejected = detector.detectMarkers(frame)
        

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            # print(f"corners={corners}")
            for c, i in zip(corners, ids.flatten()):
                pts = c.reshape(4, 2)
            msg = String()
            msg.data = ",".join([str(p) for p in pts.flatten()])
            self.publisher.publish(msg)

        cv2.imshow("Camera", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Points_img()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()