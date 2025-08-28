# file: offboard_attitude.py
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode

def quat_from_euler(roll, pitch, yaw):
    """Create a geometry_msgs/Quaternion from RPY (radians)."""
    cy = math.cos(yaw * 0.5);  sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5);sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr*cp*cy + sr*sp*sy
    q.x = sr*cp*cy - cr*sp*sy
    q.y = cr*sp*cy + sr*cp*sy
    q.z = cr*cp*sy - sr*sp*cy
    return q

class OffboardAttitude(Node):
    def __init__(self):
        super().__init__('offboard_attitude')

        # User params
        self.declare_parameter('roll_deg', 0.0)    # positive = right wing down
        self.declare_parameter('pitch_deg', 0.0)   # positive = nose up
        self.declare_parameter('yaw_deg', 0.0)     # absolute yaw (ENU) setpoint
        self.declare_parameter('thrust', 0.5)      # 0..1; ~0.5 often hovers in SITL

        self.roll = math.radians(self.get_parameter('roll_deg').value)
        self.pitch = math.radians(self.get_parameter('pitch_deg').value)
        self.yaw = math.radians(self.get_parameter('yaw_deg').value)
        self.thrust = float(self.get_parameter('thrust').value)

        # Publishers / services
        self.pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services
        for cli in (self.arm_cli, self.mode_cli):
            self.get_logger().info(f'Waiting for {cli.srv_name}...')
            cli.wait_for_service()

        # 50 Hz setpoint stream
        self.timer = self.create_timer(0.02, self.tick)
        self.startup_ticks = 0
        self.sent_offboard = False
        self.sent_armed = False

        self.get_logger().info('Streaming AttitudeTarget at 50 Hz...')

    def tick(self):
        # 1) Publish AttitudeTarget
        msg = AttitudeTarget()
        # type_mask bits: ignore body rates (1|2|4 = 7); use attitude + thrust
        msg.type_mask = 7
        msg.orientation = quat_from_euler(self.roll, self.pitch, self.yaw)
        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = 0.0
        msg.thrust = float(self.thrust)  # 0..1 for multicopter thrust setpoint
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = OffboardAttitude()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
