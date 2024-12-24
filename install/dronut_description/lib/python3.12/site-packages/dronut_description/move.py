import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy

class BirotorDroneController(Node):
    def __init__(self):
        super().__init__('birotor_drone_controller')

        self.joint_limits = {
            'slider_front': {'lower': 0.015, 'upper': 0.041},
            'slider_back': {'lower': -0.019, 'upper': 0.008},
            'slider_left': {'lower': 0.015, 'upper': 0.041},
            'slider_right': {'lower': -0.018, 'upper': 0.009},
        }

        self.joint_states = {
            'slider_front': 0.015 + (0.041 - 0.015) / 2,
            'slider_back': -0.019 + (0.008 - -0.019) / 2,
            'slider_left': 0.015 + (0.041 - 0.015) / 2,
            'slider_right': -0.018 + (0.009 - -0.018) / 2,
        }

        self.propeller_velocities = {
            'top_prop_continuous': 0.0,
            'bottom_prop_continuous': 0.0,
        }

        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info("Birotor Drone Controller Node Initialized")

    def joy_callback(self, msg):
        axes = msg.axes
        #self.get_logger().info(f"Received axes: {axes}")

        # Assuming the axes are ordered as follows (based on typical joystick setups):
        # axes[1] -> Left Stick Y (up/down) - used for ascend and descend
        # axes[3] -> Right Stick X (left/right) - used for left and right
        # axes[4] -> Right Stick Y (up/down) - used for forward and backward

        if axes[3] < -0.1:
            self.move_right()
            self.set_propellers_spin(rate=0.8)
            self.get_logger().info("Moving right")
        elif axes[3] > 0.1:
            self.move_left()
            self.set_propellers_spin(rate=0.8)
            self.get_logger().info("Moving left")
        elif axes[4] > 0.1:
            self.move_forward()
            self.set_propellers_spin(rate=0.8)
            self.get_logger().info("Moving forward")
        elif axes[4] < -0.1:
            self.move_backward()
            self.set_propellers_spin(rate=0.8)
            self.get_logger().info("Moving backward")
        elif axes[1] > 0.1:
            self.set_sliders_max()
            self.set_propellers_spin(rate=20)
            self.get_logger().info("Ascending")
        elif axes[1] < -0.1:
            self.set_sliders_min()
            self.set_propellers_spin(rate=20)
            self.get_logger().info("Descending")
        else:
            self.set_sliders_mid()
            self.set_propellers_spin(rate=0.0)
            self.get_logger().info("Stopped")

        self.publish_joint_states()


    def set_sliders_max(self):
        for joint, limits in self.joint_limits.items():
            self.joint_states[joint] = limits['upper']

    def set_sliders_min(self):
        for joint, limits in self.joint_limits.items():
            self.joint_states[joint] = limits['lower']

    def set_sliders_mid(self):
        for joint, limits in self.joint_limits.items():
            self.joint_states[joint] = limits['lower'] + (limits['upper'] - limits['lower']) / 2

    def move_forward(self):
        self.set_sliders_mid()
        self.joint_states['slider_back'] = self.joint_limits['slider_back']['upper']

    def move_backward(self):
        self.set_sliders_mid()
        self.joint_states['slider_front'] = self.joint_limits['slider_front']['upper']

    def move_left(self):
        self.set_sliders_mid()
        self.joint_states['slider_right'] = self.joint_limits['slider_right']['upper']

    def move_right(self):
        self.set_sliders_mid()
        self.joint_states['slider_left'] = self.joint_limits['slider_left']['upper']

    def set_propellers_spin(self, rate):
        self.propeller_velocities['top_prop_continuous'] = rate
        self.propeller_velocities['bottom_prop_continuous'] = rate

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = list(self.joint_states.keys()) + list(self.propeller_velocities.keys())

        propeller_positions = [
            (self.get_clock().now().nanoseconds * self.propeller_velocities['top_prop_continuous'] * 1e-9) % (2 * 3.14159),
            (self.get_clock().now().nanoseconds * self.propeller_velocities['bottom_prop_continuous'] * 1e-9) % (2 * 3.14159),
        ]

        joint_state_msg.position = list(self.joint_states.values()) + propeller_positions
        joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0] + list(self.propeller_velocities.values())

        self.joint_state_publisher.publish(joint_state_msg)



def main(args=None):
    rclpy.init(args=args)
    node = BirotorDroneController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
