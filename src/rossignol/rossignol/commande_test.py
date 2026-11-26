import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, time


ser = serial.Serial("/dev/ttyACM0", 115200)        

class Commande_test (Node) :
    def __init__ (self) :
        super().__init__('commande_test')
        self.publisher = self.create_publisher(String, 'commande_roue', 10)
        
        self.sub_calib = self.create_subscription(String, 'calibration', self.calib_cb, 10)


    def timer_callback(self) :
        msg = String()
        cg = 50
        cd = 20
        vg = cg * 0.008
        vd = cd * 0.008

        msg.data = f"{vg},{int(vd)}"
        ser.write(f"{int(cg)},{int(cd)}\n".encode())

        self.publisher.publish(msg)

    def calib_cb(self, msg) : 
        time.sleep(2) 
        self.timer = self.create_timer(0.01, self.timer_callback)


def main():
    rclpy.init()
    node = Commande_test()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()