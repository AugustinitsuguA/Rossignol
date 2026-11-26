import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

d = 0.1
global z
z = [-1,-1,-1]

def wrap(angle):
    # ramene un angle dans [-pi, pi]
    return math.atan2(math.sin(angle), math.cos(angle))

# noeud du filtre
class Filtre(Node) : 
    def __init__ (self) : 
        self.X = np.array([-0.4 , 1.5 , 0])

        self.P = np.array([[1 , 0 , 0],[0 , 1 , 0],[0 , 0 , 1]])

        

        super().__init__('filtre')
        self.publisher = self.create_publisher(Pose2D, 'position_filtree', 10)
        self.pub_pos_cmd = self.create_publisher(Pose2D, 'position_commande', 10)
        self.sub_pos = self.create_subscription(Pose2D, 'position_arUco_plan', self.cb_position, 10)
        self.sub_cmd = self.create_subscription(String, 'commande_roue', self.Kalman, 10)

    """   
    def moyenne(self, msg) :
        # avec les commandes des roues on obtient la vitesse linéaire et la vitesse angulaire
        out = msg.data.split(",")
        vg = float(out[0])
        vd = float(out[1])
        v = (vg+vd)/2
        w = (vd-vg)/d
        

        # fréquence d'envoie des commandes en sortie du réseau de neurones 
        delta_t = 0.05 

        
        v = msg.data[0]
        w = msg.data[1]
        

        # pose estimée via la caméra
        x_cam = z[0]
        y_cam = z[1]
        theta_cam = z[2]

        # positions calculée via l'odométrie : (aurait pu être un noeud à part)
        odomT = self.theta + w*delta_t
        odomX = self.x + v*math.cos(self.theta)*delta_t
        odomY = self.y + v*math.sin(self.theta)*delta_t

        pose_msg_cmd = Pose2D()
        pose_msg_cmd.x = odomX
        pose_msg_cmd.y = odomY
        pose_msg_cmd.theta = odomT
        self.pub_pos_cmd.publish(pose_msg_cmd)


        # si l aruco n'est pas visible alors z = -1 pour toutes ses composantes
        # les données de la caméras ne seront pas prises en compte
        # sinon on calcule les nouvelles coordonées en faisant la moyenne odométrie - vision
        if (z[0] != -1 and z[1] != -1) :
            self.theta = (odomT + theta_cam)/2 
            self.x = (odomX + x_cam)/2
            self.y = (odomY + y_cam)/2

        else : 
            self.theta = odomT
            self.x = odomX
            self.y = odomY

        

 
        # publication des coordonnées
        pose_msg = Pose2D()
        pose_msg.x = float(self.x)
        pose_msg.y = float(self.y)
        pose_msg.theta = float(self.theta)
        self.publisher.publish(pose_msg)
    """

    def cb_position(self, msg) : 
        # lorsqu'on recoit une position via la caméra
        global z
        z = np.array([msg.x, msg.y, msg.theta])

    def Kalman (self, msg) :
        # avec les commandes des roues on obtient la vitesse linéaire et la vitesse angulaire
        out = msg.data.split(",")
        vg = float(out[0])
        vd = float(out[1])
        v = (vg+vd)/2
        w = (vd-vg)/d

        
        # fréquence d'envoie des commandes en sortie du réseau de neurones 
        delta_t = 0.01 

        # Matrices :
        C = np.array([[1 , 0 , 0],
                      [0 , 1 , 0],
                        [0 , 0 , 1]])
        
        R = np.array([[0.005 , 0 , 0],
                      [0 , 0.005 , 0],
                        [0 , 0 , 0.005]])

        Q = np.array([[0.01 , 0 , 0],
                      [0 , 0.01 , 0],
                        [0 , 0 , 0.01]])

        # x(k-1)
        xkm1 = self.X[0]
        ykm1 = self.X[1]
        tkm1 = self.X[2]

        # evolution modèle d'état x-       
        xm = xkm1 + v*math.cos(tkm1)*delta_t
        ym = ykm1 + v*math.sin(tkm1)*delta_t
        tm = tkm1 + w*delta_t

        pose_msg_cmd = Pose2D()
        pose_msg_cmd.x = xm
        pose_msg_cmd.y = ym
        pose_msg_cmd.theta = tm
        self.pub_pos_cmd.publish(pose_msg_cmd)

        Xm = np.array([xm, ym, tm])

        F = np.array([[1 , 0 , -v*math.sin(tkm1)*delta_t],
                      [0 , 1 , v*math.cos(tkm1)*delta_t],
                        [0 , 0 , 1]])

        Pm = F @ self.P @ F.T + Q

        # si l aruco n'est pas visible alors z = -1 pour toutes ses composantes
        # les données de la caméras ne seront pas prises en compte
        # sinon on calcule les nouvelles coordonées en faisant un filtre de Kalman
        if (z[0] != -1 and z[1] != -1) :

            # pose estimée via la caméra
            Y = np.array([z[0], z[1], z[2]])


            # gain de Kalman
            K = Pm @ C.T @ np.linalg.inv(C @ Pm @ C.T + R)

            self.X = Xm + K @ (Y - C @ Xm)
            
            self.P = Pm - K @ C @ Pm

        else : 
            
            self.X = Xm
            self.P = Pm

        # publication des coordonnées
        pose_msg = Pose2D()
        pose_msg.x = float(self.X[0])
        pose_msg.y = float(self.X[1])
        pose_msg.theta = float(self.X[2])
        self.publisher.publish(pose_msg)
        





def main():
    rclpy.init()
    node = Filtre()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()