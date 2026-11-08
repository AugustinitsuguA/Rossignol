from detect import Point_plan
from detect import Point_im
import rclpy
import cv2
import time
from rclpy.node import Node
from std_msgs.msg import String

"""
a faire : faire en sorte de changer les constante à un seul endroit dans calibration
et les reperes de calib, qui sont definis à deux endroits
"""
# constantes dans l'image
cX = 1280//2
cY = 720//2
Yuco0 = 0
him0 = -30

# Détection de marqueurs ArUco avec OpenCV
dico = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dico, params)

# repères dans le plan réel
r1  = Point_plan (1.3, 0.5)
r2  = Point_plan (2.3, 0.5)
r3  = Point_plan (1.3, -0.5)
r4  = Point_plan (2.3, -0.5)

# coordonnées des points de repère dans l'image
p_calib1 = Point_im (cY+100, cX + 300)
p_calib2 = Point_im (cY-100, cX + 200)
p_calib3 = Point_im (cY+100, cX - 300)
p_calib4 = Point_im (cY-100, cX - 200)

# URL de la video via DroidCam. Pour résoudre le pb de l'accès à la caméra via WSL
URL = "http://192.168.0.20:4747/video"
# noeud ROS
class Points_img(Node) :
    def __init__ (self) : 
        super().__init__('points_img')
        self.publisher = self.create_publisher(String, 'position_arUco_img', 10)
        self.subscription = self.create_subscription(String, 'calibration', self.calib_cb, 10)

    def calib_cb(self, msg) :
        time.sleep(1)
        print(f"Calibration reçue : {msg.data}")
        self.cap = cv2.VideoCapture(URL)
        timer = 0.01
        self.timer = self.create_timer(timer, self.timer_callback)

    def timer_callback(self) :
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Image non reçue')
            return
        corners, ids, rejected = detector.detectMarkers(frame)

        cv2.circle(frame, (p_calib1.xim, p_calib1.yim), 10, (0, 255, 0), -1)
        cv2.circle(frame, (p_calib2.xim, p_calib2.yim), 10, (0, 255, 0), -1)
        cv2.circle(frame, (p_calib3.xim, p_calib3.yim), 10, (0, 255, 0), -1)
        cv2.circle(frame, (p_calib4.xim, p_calib4.yim), 10, (0, 255, 0), -1)

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