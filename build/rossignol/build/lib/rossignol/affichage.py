import rclpy
import cv2
import time
from rclpy.node import Node
from std_msgs.msg import String
from rossignol.calibration import Point_plan, Point_im
from rossignol.calibration import profon1, profon2, largeur, cX, cY

# repères dans le plan réel
r1  = Point_plan (profon1, largeur/2)
r2  = Point_plan (profon2, largeur/2)
r3  = Point_plan (profon1, -largeur/2)
r4  = Point_plan (profon2, -largeur/2)

r21  = Point_plan (profon1, largeur/2 + 1)
r23  = Point_plan (profon1, -largeur/2 - 1)

URL = "http://192.168.0.21:4747/video"
# noeud ROS
class Affichage(Node) :
    def __init__ (self) : 
        super().__init__('affichage')
        self.sub_calib = self.create_subscription(String, 'calibration', self.calib_cb, 10)
        self.sub_point_im = self.create_subscription(String, 'position_arUco_img', self.pos_im, 10)
        self.sub_filtre = self.create_subscription(String, 'position_filtree', self.get_filtre, 10)

    def calib_cb(self, msg) :
        time.sleep(1)
        # print(f"Calibration reçue : {msg.data}")
        message = msg.data.split(",")
        global y0, K, f, K1
        y0 = float(message[0])
        K = float(message[1])
        f = float(message[2])
        K1 = float(message[3])
        self.cap = cv2.VideoCapture(URL)
        timer = 1/30
        self.timer = self.create_timer(timer, self.timer_callback)

    def timer_callback(self) :
        1==1
        """ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Image non reçue')
            return

        cv2.circle(frame, (r1.plan_2_im().xim, r1.plan_2_im().yim), 10, (0, 255, 0), -1)
        cv2.circle(frame, (r2.plan_2_im().xim, r2.plan_2_im().yim), 10, (0, 255, 0), -1)
        cv2.circle(frame, (r3.plan_2_im().xim, r3.plan_2_im().yim), 10, (0, 255, 0), -1)
        cv2.circle(frame, (r4.plan_2_im().xim, r4.plan_2_im().yim), 10, (0, 255, 0), -1)

        cv2.line(frame, (0, int(y0)), (1200, int(y0)), (100, 100, 255), 2)

        cv2.line(frame, (cX, int(y0)), (int(r1.plan_2_im().xim), int(r1.plan_2_im().yim)), (100, 100, 255), 2)
        cv2.line(frame, (cX, int(y0)), (int(r3.plan_2_im().xim), int(r3.plan_2_im().yim)), (100, 100, 255), 2)

        cv2.line(frame, (cX, int(y0)), (int(r21.plan_2_im().xim), int(r21.plan_2_im().yim)), (100, 100, 255), 2)
        cv2.line(frame, (cX, int(y0)), (int(r23.plan_2_im().xim), int(r23.plan_2_im().yim)), (100, 100, 255), 2)

        cv2.imshow("Camera", frame)
        cv2.waitKey(1)"""

    def pos_im(self, msg) :
        print("pos_im")

    def get_filtre(self, msg) :
        print("get_filtre")
    


def main(args=None):
    rclpy.init(args=args)
    node = Affichage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()