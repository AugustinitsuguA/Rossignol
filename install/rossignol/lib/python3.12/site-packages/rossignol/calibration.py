import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from std_msgs.msg import String

# 2 choses à régler pour calibrer la caméra : les coordonées réel et dans l'image des points de repère
# repères dans le plan réel
profon1 = 1.3
profon2 = 2.3
largeur = 1

pxy1 = 100
pxy2 = 50
pxx1 = 350
pxx2 = 280


# URL de la video via DroidCam. Pour résoudre le pb de l'accès à la caméra via WSL
URL = "http://192.168.0.21:4747/video"

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
    
    def afficher(self, frame) :
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



def calibration_const(profon1, profon2, largeur, pxy1, pxy2, pxx1, pxx2) :
    global r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14, r15, r16, r17, r18, p_calib1, p_calib2
    global y0, f, K, K1
    # -------------------------------------------------------------
    r1  = Point_plan (profon1, largeur/2)
    r2  = Point_plan (profon2, largeur/2)
    r3  = Point_plan (profon1, -largeur/2)
    r4  = Point_plan (profon2, -largeur/2)

    # coordonnées des points de repère dans l'image
    p_calib1 = Point_im (cY+pxy1, cX + pxx1)
    p_calib2 = Point_im (cY-pxy2, cX + pxx2)
    p_calib3 = Point_im (cY+pxy1, cX - pxx1)
    p_calib4 = Point_im (cY-pxy2, cX - pxx2)

    # ---------------------------------------------------------------

    r5  = Point_plan (profon1, 1.5)
    r6  = Point_plan (profon1, -1.5)

    r7  = Point_plan (profon1, 2.5)
    r8  = Point_plan (profon1, -2.5)

    r9  = Point_plan (profon2, 2.5)
    r10  = Point_plan (profon2, -2.5)

    r11  = Point_plan (profon2+1, 2.5)
    r12  = Point_plan (profon2+1, -2.5)

    r13  = Point_plan (profon2+2, 2.5)
    r14  = Point_plan (profon2+2, -2.5)

    r15  = Point_plan (profon2+3, 2.5)
    r16  = Point_plan (profon2+3, -2.5)

    r17  = Point_plan (profon2+4, 2.5)
    r18  = Point_plan (profon2+4, -2.5)






    r = Point_plan (1.8, 0.5) # point test

    ob1 = obstacle (1.8, 1.1)
    ob2 = obstacle (1.5, 0.8)

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

class Calibration(Node) :
    def __init__ (self) :
        super().__init__('calibration')
        self.publisher = self.create_publisher(String, 'calibration', 10)
        self.pxx1 = pxx1
        self.pxx2 = pxx2
        self.pxy1 = pxy1

        calibration_const(
            profon1, profon2, largeur, self.pxy1, pxy2, self.pxx1, self.pxx2
        )

        self.cap = cv2.VideoCapture(URL)
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError(f"Impossible d'ouvrir la caméra : {URL}")

        self.timer = self.create_timer(1 / 30, self.timer_callback)

    def recalculate(self):
        calibration_const(
            profon1, profon2, largeur, self.pxy1, pxy2, self.pxx1, self.pxx2
        )

    def stop_preview(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        self.cap.release()
        cv2.destroyAllWindows()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Image non reçue')
            self.stop_preview()
            return

        cv2.circle(frame, (r1.plan_2_im().xim, r1.plan_2_im().yim), 10, (0, 255, 0), -1)
        cv2.circle(frame, (r2.plan_2_im().xim, r2.plan_2_im().yim), 10, (0, 255, 0), -1)
        cv2.circle(frame, (r3.plan_2_im().xim, r3.plan_2_im().yim), 10, (0, 255, 0), -1)
        cv2.circle(frame, (r4.plan_2_im().xim, r4.plan_2_im().yim), 10, (0, 255, 0), -1)
        cv2.circle(frame, (p_calib1.xim, p_calib1.yim), 10, (255, 255, 0), -1)
        cv2.circle(frame, (p_calib2.xim, p_calib2.yim), 10, (255, 255, 0), -1)

        cv2.line(frame, (0, int(y0)), (1200, int(y0)), (255, 100, 100), 1)
        cv2.line(frame, (cX, int(y0)), (int(r1.plan_2_im().xim), int(r1.plan_2_im().yim)), (100, 100, 255), 1)
        cv2.line(frame, (cX, int(y0)), (int(r3.plan_2_im().xim), int(r3.plan_2_im().yim)), (100, 100, 255), 1)
        cv2.line(frame, (cX, int(y0)), (int(r5.plan_2_im().xim), int(r5.plan_2_im().yim)), (100, 100, 255), 1)
        cv2.line(frame, (cX, int(y0)), (int(r6.plan_2_im().xim), int(r6.plan_2_im().yim)), (100, 100, 255), 1)
        cv2.line(frame, (cX, int(y0)), (int(r7.plan_2_im().xim), int(r7.plan_2_im().yim)), (100, 100, 255), 1)
        cv2.line(frame, (cX, int(y0)), (int(r8.plan_2_im().xim), int(r8.plan_2_im().yim)), (100, 100, 255), 1)
        cv2.line(frame, (int(r9.plan_2_im().xim), int(r9.plan_2_im().yim)), (int(r10.plan_2_im().xim), int(r10.plan_2_im().yim)), (100, 100, 255), 1)
        cv2.line(frame, (int(r7.plan_2_im().xim), int(r7.plan_2_im().yim)), (int(r8.plan_2_im().xim), int(r8.plan_2_im().yim)), (100, 100, 255), 1)
        cv2.line(frame, (int(r11.plan_2_im().xim), int(r11.plan_2_im().yim)), (int(r12.plan_2_im().xim), int(r12.plan_2_im().yim)), (100, 100, 255), 1)
        cv2.line(frame, (int(r13.plan_2_im().xim), int(r13.plan_2_im().yim)), (int(r14.plan_2_im().xim), int(r14.plan_2_im().yim)), (100, 100, 255), 1)

        cv2.imshow("Calibration", frame)
        key = cv2.waitKey(1)

        if key in (ord('a'), ord('A')):
            self.pxx1 += 10
            self.recalculate()
        elif key in (ord('z'), ord('Z')):
            self.pxx1 -= 10
            self.recalculate()
        elif key in (ord('e'), ord('E')):
            self.pxx2 += 10
            self.recalculate()
        elif key in (ord('r'), ord('R')):
            self.pxx2 -= 10
            self.recalculate()
        elif key in (ord('q'), ord('Q')):
            self.pxy1 += 10
            self.recalculate()
        elif key in (ord('s'), ord('S')):
            self.pxy1 -= 10
            self.recalculate()
        elif key in (ord('y'), ord('Y')):
            msg = String()
            msg.data = f"{y0},{K},{f},{K1}"
            self.publisher.publish(msg)
            print(f"Calibration envoyée : y0={y0}, K={K}, f={f}, K1={K1}")
            print(frame.shape)
            self.stop_preview()

    def destroy_node(self):
        self.stop_preview()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Calibration()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
