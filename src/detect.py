import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import time
import serial
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose2D


rclpy.init()
node = Node('detect')
pub = node.create_publisher(Pose2D, 'position_plan', 10)


# ser = serial.Serial("COM11", 115200)   # adapter le port
cX = 300
Yuco0 = 0

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
        cv2.circle(frame, (point_im.xim, point_im.yim), 5, (200, 0, 200), -1)
        # print(f"Z={self.Z} X={self.X} => x={point_im.xim} y={point_im.yim}")


class Point_im : 
    def __init__ (self, yim, xim = 0) : 
        self.yim = yim
        self.xim = xim

    def im_2_plan(self) :
        Z = K/(self.yim - y0)
        X = (self.xim - cX) * Z / f
        return Point_plan(Z, X)

    def imUco_2_plan(self, him0 = -30) :
        K1 = him0 * r1.Z
        Z = (K + K1) / (self.yim - y0)
        X = (self.xim - cX) * (K + K1) / (self.yim - y0) / f
        return Point_plan(Z, X)

def orientation_plan(pts):
    P0 = Point_im(pts[0][1], pts[0][0]).imUco_2_plan()
    P1 = Point_im(pts[1][1], pts[1][0]).imUco_2_plan()
    dZ = P1.Z - P0.Z
    dX = P1.X - P0.X
    return np.arctan2(dZ, dX) 

plt.ion()
fig, ax = plt.subplots()
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(0, 3)
ax.set_aspect("equal")
ax.set_xlabel("X (m)")
ax.set_ylabel("Z (m)")
ax.grid(True)

robot, = ax.plot([], [], "bo", markersize=10)

xs = [-0.5, 0.5, 0.5, -0.5, -0.5]
zs = [1.3, 1.3, 2.3, 2.3, 1.3]
ax.plot(xs, zs, "r-")

r1  = Point_plan (1.3, 0.5)
r2  = Point_plan (2.3, 0.5)
r3  = Point_plan (1.3, -0.5)
r4  = Point_plan (2.3, -0.5)

ob1 = obstacle (1.8, 1.1)
ob2 = obstacle (1.5, 0.8)

p_calib1 = Point_im (400, 453)
p_calib2 = Point_im (300, 386)
p_calib3 = Point_im (400, 146)
p_calib4 = Point_im (300, 213)

A = np.array([[r1.Z, 1], [r2.Z, 1]])
b = np.array([r1.Z * p_calib1.yim, r2.Z * p_calib2.yim])

X = np.linalg.solve(A, b)
y0 = X[0]
K = X[1]
f = (p_calib1.xim - p_calib3.xim) * r1.Z / (r1.X - r3.X)

print(y0)
print(K)
print(f)

# Détection de marqueurs ArUco avec OpenCV
dico = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dico, params)

# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture("video_test.mp4") 
while True :
    ret, frame = cap.read() 
    corners, ids, rejected = detector.detectMarkers(frame)

    r1.afficher()
    r2.afficher()
    r3.afficher()
    r4.afficher()

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        # print(f"corners={corners}")
        for c, i in zip(corners, ids.flatten()):
            pts = c.reshape(4, 2)
            u, v = pts.mean(axis=0)
        Yuco = v
        point_Uco = Point_im(Yuco, u)
        robot_plan = point_Uco.imUco_2_plan()
        orientation = orientation_plan(pts)
        print(f"1 : Z = {robot_plan.Z:.2f}, X = {robot_plan.X:.2f}, Orientation = {orientation:.2f} rad = ({np.degrees(orientation):.2f} deg)")

        # envoyer(robot_plan.Z*30, robot_plan.X*30)  # envoyer les coordonnées au robot

        # mise a jour du plot
        robot.set_data([robot_plan.X], [robot_plan.Z])

        # publication des coordonnées sur le topic ROS
        pose_msg = Pose2D()
        pose_msg.x = float(robot_plan.X)
        pose_msg.y = float(robot_plan.Z)
        pose_msg.theta = float(orientation)
        pub.publish(pose_msg)


    cv2.imshow("Camera", frame)


    if (cv2.waitKey(1) & 0xFF == ord("q")):
        break

    time.sleep(0.1)


cv2.destroyAllWindows()
#ser.close()