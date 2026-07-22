from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from singulator_interfaces.msg import MatrixCommand, MatrixState

class MatrixCommandFanout(Node):
    def __init__(self):
        super().__init__("matrix_command_fanout")
        self.declare_parameter("rows",14); self.declare_parameter("cols",4)
        self.rows=int(self.get_parameter("rows").value); self.cols=int(self.get_parameter("cols").value)
        self.cell_count=self.rows*self.cols
        self.cell_publishers: List = []
        for row in range(self.rows):
            for col in range(self.cols):
                topic=f"/singulator/cell/r{row:02d}_c{col:02d}/cmd_vel"
                self.cell_publishers.append(self.create_publisher(Float64,topic,10))
        self.state_pub=self.create_publisher(MatrixState,"/singulator/matrix/state",10)
        self.subscription=self.create_subscription(MatrixCommand,"/singulator/matrix/command",self.on_command,10)
        self._logged_first_command=False
        self.get_logger().info(f"Matrix fan-out ready for {self.rows}x{self.cols} cells")

    def on_command(self,msg):
        if msg.rows!=self.rows or msg.cols!=self.cols:
            self.get_logger().debug(f"Matrix dimensions mismatch: got {msg.rows}x{msg.cols}")
            return
        if len(msg.target_speed_mps)!=self.cell_count:
            self.get_logger().error(f"Expected {self.cell_count} speeds, got {len(msg.target_speed_mps)}")
            return
        if not self._logged_first_command:
            values=[float(value) for value in msg.target_speed_mps]
            self.get_logger().info(
                "First matrix command accepted: "
                f"min={min(values):.3f} m/s, max={max(values):.3f} m/s, "
                f"count={len(values)}"
            )
            self._logged_first_command=True
        for pub,speed in zip(self.cell_publishers,msg.target_speed_mps):
            cell=Float64(); cell.data=float(speed); pub.publish(cell)
        state=MatrixState(); state.header=msg.header; state.rows=self.rows; state.cols=self.cols
        state.target_speed_mps=list(msg.target_speed_mps)
        state.actual_speed_mps=list(msg.target_speed_mps)
        state.fault=[False]*self.cell_count
        self.state_pub.publish(state)

def main(args=None):
    rclpy.init(args=args)
    node=MatrixCommandFanout()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
