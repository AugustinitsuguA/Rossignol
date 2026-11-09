import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from std_msgs.msg import String

# noeud de calibration
class Calibration(Node) : 
    def __init__ (self) : 

        super().__init__('calibration')
        self.publisher = self.create_publisher(String, 'calibration', 10)



# URL de la video via DroidCam. Pour résoudre le pb de l'accès à la caméra via WSL
URL = "http://192.168.0.20:4747/video"

# constantes dans l'image
global y0, K, f, K1
cX = 1280//2
cY = 720//2
Yuco0 = 0
him0 = -60 # taille en px du robot quand il est sur la première ligne

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

r = Point_plan (1.8, 0.5) # point test

ob1 = obstacle (1.8, 1.1)
ob2 = obstacle (1.5, 0.8)

# coordonnées des points de repère dans l'image
p_calib1 = Point_im (cY+100, cX + 350)
# print(p_calib1.yim, p_calib1.xim)
p_calib2 = Point_im (cY-50, cX + 280)
# print(p_calib2.yim, p_calib2.xim)
p_calib3 = Point_im (cY+100, cX - 350)
# print(p_calib3.yim, p_calib3.xim)
p_calib4 = Point_im (cY-50, cX - 280)
# print(p_calib4.yim, p_calib4.xim)

A = np.array([[r1.Z, 1], [r2.Z, 1]])
b = np.array([r1.Z * p_calib1.yim, r2.Z * p_calib2.yim])

# calcul des constantes
X = np.linalg.solve(A, b)
y0 = X[0]
K = X[1]
f = (p_calib1.xim - p_calib3.xim) * r1.Z / (r1.X - r3.X)   
K1 = him0 * r1.Z


print("---")
print(y0)
print(K)
print(f)
print(K1)



# Tant que l'utilisateur n'a pas appuyé sur Y, no affiche l'image avec les points repères.
# Lorsqu'il appuie sur Y, on envoie les constante y0, K et f sur le topic de calibration. 
# Ces données sont envoyées une seule fois au début, ce noeud sert de calibration initiale.
# Une fois que la calibration envoyées, les autres noeuds se lancent. 
cap = cv2.VideoCapture(URL)
rclpy.init()
node = Calibration()
while True : 
    ret, frame = cap.read()
    if not ret : 
        print("Image non reçue")
        break
    
    cv2.circle(frame, (r1.plan_2_im().xim, r1.plan_2_im().yim), 10, (0, 255, 0), -1)
    cv2.circle(frame, (r2.plan_2_im().xim, r2.plan_2_im().yim), 10, (0, 255, 0), -1)
    cv2.circle(frame, (r3.plan_2_im().xim, r3.plan_2_im().yim), 10, (0, 255, 0), -1)
    cv2.circle(frame, (r4.plan_2_im().xim, r4.plan_2_im().yim), 10, (0, 255, 0), -1)

    cv2.line(frame, (0, int(y0)), (1200, int(y0)), (100, 100, 255), 2)

    cv2.line(frame, (cX, int(y0)), (int(r1.plan_2_im().xim), int(r1.plan_2_im().yim)), (100, 100, 255), 2)
    cv2.line(frame, (cX, int(y0)), (int(r3.plan_2_im().xim), int(r3.plan_2_im().yim)), (100, 100, 255), 2)

    cv2.line(frame, (cX, int(y0)), (int(r21.plan_2_im().xim), int(r21.plan_2_im().yim)), (100, 100, 255), 2)
    cv2.line(frame, (cX, int(y0)), (int(r23.plan_2_im().xim), int(r23.plan_2_im().yim)), (100, 100, 255), 2)

    cv2.imshow("Calibration", frame)
    key = cv2.waitKey(1)
    if key == ord('y') or key == ord('Y'):

        msg = String()
        msg.data = f"{y0},{K},{f},{K1}"
        node.publisher.publish(msg)
        
        print(f"Calibration envoyée : y0={y0}, K={K}, f={f}, K1={K1}")
        print (frame.shape)

        
        break
