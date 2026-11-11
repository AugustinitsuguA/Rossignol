import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

d = 10

# noeud du filtre
class Filtre(Node) : 
    def __init__ (self) : 

        super().__init__('filtre')
        self.publisher = self.create_publisher(String, 'position_filtree', 10)
        self.sub_pos = self.create_subscription(Pose2D, 'position_arUco_plan', self.cb_position, 10)
        self.sub_cmd = self.create_subscription(String, 'commande_roue', self.cb_cmd, 10)

    def cb_cmd(self, msg) :
        # avec les commandes des roues on obtient la vitesse linéaire et la vitesse angulaire
        vg = msg.data[0]
        vd = msg.data[1]
        self.v = (vg+vd)/2
        self.w = (vd-vg)/d

        # x = xp + ...
        # faire ca avec y et theta
        # faire moyenne avec position via caméra
        # trouver un moyen de sauvegarder xp yp thetap
        # trouver une solution quand on a pas accès aux infos de la caméra

    def cb_position(self, msg) : 
        # lorsqu'on recoit une position via la caméra
        z = np.array([msg.x, msg.y, msg.theta])





def main():
    rclpy.init()
    rclpy.spin(Filtre())
    rclpy.shutdown()


if __name__ == '__main__':
    main()