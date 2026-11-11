import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import time
import serial
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose2D

# constantes dans l'image
cX = 1280//2
cY = 720//2

def envoyer(vg, vd):
    vg = int(np.clip(vg, -100, 100))
    vd = int(np.clip(vd, -100, 100))
    # ser.write(f"{vg},{vd}\n".encode())


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

# pour avoir l'orientation de l'arUco dan le plan
# pts sont les coord de deux points de l'arUco
# ils sont mis dans le plan puis trigo pour trouver l'orientation
def orientation_plan(pts):
    P0 = Point_im(pts[0][1], pts[0][0]).imUco_2_plan()
    P1 = Point_im(pts[1][1], pts[1][0]).imUco_2_plan()
    dZ = P1.Z - P0.Z
    dX = P1.X - P0.X
    return np.arctan2(dZ, dX) 


# noeud ROS

class Detect(Node) : 
    def __init__ (self) : 
        super().__init__('detect')
        self.publisher = self.create_publisher(Pose2D, 'position_arUco_plan', 10)
        self.sub_pos = self.create_subscription(String, 'position_arUco_img', self.callback, 10)
        self.sub_calib = self.create_subscription(String, 'calibration', self.calib_cb, 10)
        
    def calib_cb(self, msg) :
        message = msg.data.split(",")
        global y0, K, f, K1
        y0 = float(message[0])
        K = float(message[1])
        f = float(message[2])
        K1 = float(message[3])
        time.sleep(0.1)
   
    def callback(self, msg) :

        self.get_logger().info(f"Position reçue : {msg.data}")
        out = msg.data.split(",")
        #print(f"Position reçue : x1={out[0]}, y1={out[1]}, x2={out[2]}, y2={out[3]}, x3={out[4]}, y3={out[5]}, x4={out[6]}, y4={out[7]}")
        pts = np.array([[float(out[0]), float(out[1])], [float(out[2]), float(out[3])], [float(out[4]), float(out[5])], [float(out[6]), float(out[7])]])
        Xuco, Yuco = pts.mean(axis=0)
        print(f"Coordonnées moyennes de l'arUco : Xuco={Xuco:.2f}, Yuco={Yuco:.2f}")
        point_Uco = Point_im(Yuco, Xuco) # création du point image avec le centre du l'arUco
        robot_plan = point_Uco.imUco_2_plan() # Transformation dans le plan
        orientation = orientation_plan(pts)

        # publication des coordonnées sur le topic ROS
        pose_msg = Pose2D()
        pose_msg.x = float(robot_plan.X)
        pose_msg.y = float(robot_plan.Z)
        pose_msg.theta = float(orientation)
        self.publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Detect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

