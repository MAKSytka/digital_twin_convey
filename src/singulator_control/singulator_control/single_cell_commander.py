import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class SingleCellCommander(Node):
    def __init__(self):
        super().__init__("single_cell_commander")
        self.declare_parameter("speed_mps",0.5)
        self.declare_parameter("publish_rate_hz",20.0)
        self.publisher=self.create_publisher(Float64,"/singulator/cell/r00_c00/cmd_vel",10)
        rate=float(self.get_parameter("publish_rate_hz").value)
        if rate<=0: raise ValueError("publish_rate_hz must be positive")
        self.timer=self.create_timer(1.0/rate,self.on_timer)
        self.get_logger().info("Single-cell commander started")

    def on_timer(self):
        msg=Float64()
        msg.data=float(self.get_parameter("speed_mps").value)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node=SingleCellCommander()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
