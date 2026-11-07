import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

rclpy.init()
node = Node('filtre')
node.create_subscription(Pose2D, 'position_arUco_plan', lambda msg: print(f"Position reçue : x={msg.x}, y={msg.y}, theta={msg.theta}"), 10)
rclpy.spin(node)
