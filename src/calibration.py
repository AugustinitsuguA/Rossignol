import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Calibration(Node) : 
    def __init__ (self) : 

        super().__init__('calibration')
        self.publisher_ = self.create_publisher(String, 'calibration', 10)

def main(args=None):
    rclpy.init(args=args)
    node = Calibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()