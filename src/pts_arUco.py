import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 

# URL de la video via DroidCam. Pour résoudre le pb de l'accès à la caméra via WSL
URL = "http://192.168.0.20:4747/video?640x480"
# noeud ROS
rclpy.init()
node = Node('pts_arUco')
pub = node.create_publisher(String, 'position_arUco_img', 10)
