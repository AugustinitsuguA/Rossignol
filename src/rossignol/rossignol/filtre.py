import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

d = 10
global z
z = [-1,-1,-1]

# noeud du filtre
class Filtre(Node) : 
    def __init__ (self) : 
        self.x = -0.4
        self.y = 1.5
        self.theta = 0

        super().__init__('filtre')
        self.publisher = self.create_publisher(Pose2D, 'position_filtree', 10)
        self.pub_pos_cmd = self.create_publisher(Pose2D, 'position_commande', 10)
        self.sub_pos = self.create_subscription(Pose2D, 'position_arUco_plan', self.cb_position, 10)
        self.sub_cmd = self.create_subscription(String, 'commande_roue', self.cb_cmd, 10)

    def cb_cmd(self, msg) :
        # avec les commandes des roues on obtient la vitesse linéaire et la vitesse angulaire
        out = msg.data.split(",")
        vg = float(out[0])
        vd = float(out[1])
        v = (vg+vd)/2
        w = (vd-vg)/d
        

        # fréquence d'envoie des commandes en sortie du réseau de neurones 
        delta_t = 0.05 

        """
        v = msg.data[0]
        w = msg.data[1]
        """

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

    def cb_position(self, msg) : 
        # lorsqu'on recoit une position via la caméra
        global z
        z = np.array([msg.x, msg.y, msg.theta])





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