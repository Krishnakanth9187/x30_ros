import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from axis_msgs.msg import Ptz
import threading


class AxisPTZ(Node):
    def __init__(self, args):
        super().__init__('axis_ptz')

        self.rate = args['ptz_rate']
        self.use_control_timeout = args['use_control_timeout']
        self.control_timeout_value = args['control_timeout_value']

        self.desired_pan_vel = 0.0
        self.desired_tilt_vel = 0.0
        self.pan_angle = 0.0
        self.tilt_angle = 0.0

        if self.use_control_timeout:
            self.last_command_time = self.get_clock().now()
            self.command_timeout = Duration(seconds=self.control_timeout_value)

        self.pub_pan_tilt_velocity = self.create_publisher(Float64MultiArray, "/pan_tilt_velocity_controller/commands", 10)

        self.sub_cmd_vel = self.create_subscription(Ptz, "/cmd_vel_ptz", self.cmd_vel_callback, 10)
        self.sub_joint_states = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)

        self.home_service = self.create_service(Empty, 'home_ptz', self.home_service_callback)

        self.get_logger().info("AxisPTZ Node Initialized")

    def cmd_vel_callback(self, msg):
        """Extract pan and tilt velocities from /cmd_vel message."""
        self.desired_pan_vel = msg.pan # Assuming Z-axis rotation controls pan
        self.desired_tilt_vel = msg.tilt  # Assuming Y-axis rotation controls tilt

        # Publish to pan-tilt velocity controller
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [self.desired_pan_vel, self.desired_tilt_vel]
        self.pub_pan_tilt_velocity.publish(velocity_msg)

        self.get_logger().info(f"Published Pan/Tilt Velocities: {velocity_msg.data}")

        # Update last command time
        self.last_command_time = self.get_clock().now()

    def joint_state_callback(self, msg):
        """Extract pan and tilt joint angles from joint_states."""
        try:
            if "payload_camera_pan_joint" in msg.name and "payload_camera_tilt_joint" in msg.name:
                pan_index = msg.name.index("payload_camera_pan_joint")
                tilt_index = msg.name.index("payload_camera_tilt_joint")

                self.pan_angle = msg.position[pan_index]
                self.tilt_angle = msg.position[tilt_index]

                self.get_logger().info(f"Pan Angle: {self.pan_angle:.2f}, Tilt Angle: {self.tilt_angle:.2f}")

        except ValueError:
            self.get_logger().warn("Pan/Tilt joints not found in joint_states")

    def home_service_callback(self, request, response):
        """Reset PTZ to home position."""
        self.get_logger().info("Homing PTZ camera")
        return response

    def control_loop(self):
        """Monitor timeout and reset velocities if needed."""
        rate = self.create_rate(self.rate)
        while rclpy.ok():
            if self.use_control_timeout:
                elapsed_time = self.get_clock().now() - self.last_command_time
                if elapsed_time > self.command_timeout:
                    self.desired_pan_vel = 0.0
                    self.desired_tilt_vel = 0.0
                    velocity_msg = Float64MultiArray()
                    velocity_msg.data = [0.0, 0.0]
                    self.pub_pan_tilt_velocity.publish(velocity_msg)
                    self.get_logger().warn("Control timeout reached. Stopping PTZ movement.")

            rate.sleep()

    def run(self):
        """Start the control loop in a separate thread and spin the node."""
        thread = threading.Thread(target=self.control_loop)
        thread.start()
        rclpy.spin(self)
        thread.join()

def main():
    rclpy.init()
    args = {
        'ptz_rate': 10,
        'use_control_timeout': True,
        'control_timeout_value': 5.0,
    }
    node = AxisPTZ(args)
    node.run()
    rclpy.shutdown()





if __name__ == '__main__':
    main()
