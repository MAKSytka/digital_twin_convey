import rclpy
from rclpy.node import Node
from singulator_interfaces.msg import MatrixCommand

class UniformMatrixController(Node):
    def __init__(self):
        super().__init__("uniform_matrix_controller")
        self.declare_parameter("rows",14); self.declare_parameter("cols",4)
        self.declare_parameter("speed_mps",2.0); self.declare_parameter("publish_rate_hz",20.0)
        self.publisher=self.create_publisher(MatrixCommand,"/singulator/matrix/command",10)
        rate=float(self.get_parameter("publish_rate_hz").value)
        if rate<=0: raise ValueError("publish_rate_hz must be positive")
        speed=float(self.get_parameter("speed_mps").value)
        self.get_logger().info(
            f"Uniform matrix command: {speed:.3f} m/s for 56 cells"
        )
        self.timer=self.create_timer(1.0/rate,self.on_timer)

    def on_timer(self):
        rows=int(self.get_parameter("rows").value); cols=int(self.get_parameter("cols").value)
        speed=float(self.get_parameter("speed_mps").value)
        msg=MatrixCommand(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id="matrix"
        msg.rows=rows; msg.cols=cols; msg.target_speed_mps=[speed]*(rows*cols)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node=UniformMatrixController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
