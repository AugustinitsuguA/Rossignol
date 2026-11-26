import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

global x_aruco, y_aruco, theta_aruco, x_cmd, y_cmd, theta_cmd, x_filtre, y_filtre, theta_filtre
x_aruco = -1
y_aruco = -1
theta_aruco = -1
x_cmd = -1
y_cmd = -1
theta_cmd = -1
x_filtre = -1
y_filtre = -1
theta_filtre = -1

# noeud ROS
class Record(Node) :
    def __init__ (self) : 
        super().__init__('record')
        self.sub_aruco = self.create_subscription(Pose2D, 'position_arUco_plan', self.get_aruco, 10)
        self.sub_cmd = self.create_subscription(Pose2D, 'position_commande', self.get_cmd, 10)
        self.sub_filtre = self.create_subscription(Pose2D, 'position_filtree', self.get_filtre, 10)
        self.sub_calib = self.create_subscription(String, 'calibration', self.calib, 10)

    def calib(self, msg) :
        with open("record.txt", "w", encoding="utf-8"):
             pass
        print("création doc record")
        timer = 0.1
        self.timer = self.create_timer(timer, self.timer_callback)

    def get_aruco(self, msg) :
        global x_aruco, y_aruco, theta_aruco
        x_aruco = msg.x
        y_aruco = msg.y
        theta_aruco = msg.theta

    def get_cmd(self, msg) :
        global x_cmd, y_cmd, theta_cmd
        x_cmd = msg.x
        y_cmd = msg.y
        theta_cmd = msg.theta

    def get_filtre(self, msg) :
        global x_filtre, y_filtre, theta_filtre
        x_filtre = msg.x
        y_filtre = msg.y
        theta_filtre = msg.theta

    def timer_callback(self) :
        global x_aruco, y_aruco, theta_aruco, x_cmd, y_cmd, theta_cmd, x_filtre, y_filtre, theta_filtre
        with open("record.txt", "a", encoding="utf-8") as f:
            f.write(f"{x_aruco}, {y_aruco}, {theta_aruco}, {x_cmd}, {y_cmd}, {theta_cmd}, {x_filtre}, {y_filtre}, {theta_filtre}\n")



def main(args=None):
    rclpy.init(args=args)
    node = Record()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()