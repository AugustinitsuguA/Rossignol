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
him0 = -0

# Détection de marqueurs ArUco avec OpenCV
dico = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dico, params)

class obstacle : 
    def __init__(self, Z, X, taille = 0) :
        self.Z = Z
        self.X = X
        self.taille = taille

class Point_plan :
    def __init__(self, Z, X) :
        self.Z = Z
        self.X = X

    def plan_2_im(self) :
        x = int(cX + f*self.X/self.Z)
        y = int((K + self.Z*y0)/self.Z)
        return Point_im(y, x)    
    
    def afficher(self) :
        point_im = self.plan_2_im()
        cv2.circle(frame, (point_im.xim, point_im.yim), 10, (0, 255, 0), -1)
        # print(f"Z={self.Z} X={self.X} => x={point_im.xim} y={point_im.yim}")


class Point_im : 
    def __init__ (self, yim, xim = 0) : 
        self.yim = yim
        self.xim = xim

    def im_2_plan(self) :
        Z = K/(self.yim - y0)
        X = (self.xim - cX) * Z / f
        return Point_plan(Z, X)

    def imUco_2_plan(self) :
        Z = (K + K1) / (self.yim - y0)
        X = (self.xim - cX) * (K + K1) / (self.yim - y0) / f
        return Point_plan(Z, X)


# repères dans le plan réel
r1  = Point_plan (1.3, 0.5)
r2  = Point_plan (2.3, 0.5)
r3  = Point_plan (1.3, -0.5)
r4  = Point_plan (2.3, -0.5)

r21  = Point_plan (1.3, 1.5)
r23  = Point_plan (1.3, -1.5)

r = Point_plan (1.8, 0)

# coordonnées des points de repère dans l'image
p_calib1 = Point_im (cY+100, cX + 300)
p_calib2 = Point_im (cY-100, cX + 200)
p_calib3 = Point_im (cY+100, cX - 300)
p_calib4 = Point_im (cY-100, cX - 200)

# URL de la video via DroidCam. Pour résoudre le pb de l'accès à la caméra via WSL
# 172.20.10.1:4747
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
        message = msg.data.split(",")
        global y0, K, f, K1
        y0 = float(message[0])
        K = float(message[1])
        f = float(message[2])
        K1 = float(message[3])
        self.cap = cv2.VideoCapture(URL)
        timer = 0.01
        self.timer = self.create_timer(timer, self.timer_callback)

    def timer_callback(self) :
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Image non reçue')
            return
        corners, ids, rejected = detector.detectMarkers(frame)

        cv2.circle(frame, (r1.plan_2_im().xim, r1.plan_2_im().yim), 10, (0, 255, 0), -1)
        cv2.circle(frame, (r2.plan_2_im().xim, r2.plan_2_im().yim), 10, (0, 255, 0), -1)
        cv2.circle(frame, (r3.plan_2_im().xim, r3.plan_2_im().yim), 10, (0, 255, 0), -1)
        cv2.circle(frame, (r4.plan_2_im().xim, r4.plan_2_im().yim), 10, (0, 255, 0), -1)

        cv2.line(frame, (0, int(y0)), (1200, int(y0)), (100, 100, 255), 2)

        cv2.line(frame, (cX, int(y0)), (int(r1.plan_2_im().xim), int(r1.plan_2_im().yim)), (100, 100, 255), 2)
        cv2.line(frame, (cX, int(y0)), (int(r3.plan_2_im().xim), int(r3.plan_2_im().yim)), (100, 100, 255), 2)

        cv2.line(frame, (cX, int(y0)), (int(r21.plan_2_im().xim), int(r21.plan_2_im().yim)), (100, 100, 255), 2)
        cv2.line(frame, (cX, int(y0)), (int(r23.plan_2_im().xim), int(r23.plan_2_im().yim)), (100, 100, 255), 2)


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