import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class BirotorDroneController(Node):
    def __init__(self):
        super().__init__('birotor_drone_controller')

        # Joint limits
        self.joint_limits = {
            'slider_front': {'lower': 0.015, 'upper': 0.041},
            'slider_back': {'lower': -0.019, 'upper': 0.008},
            'slider_left': {'lower': 0.015, 'upper': 0.041},
            'slider_right': {'lower': -0.018, 'upper': 0.009},
        }

        # Initial joint states
        self.joint_states = {
            'slider_front': 0.015 + (0.041 - 0.015) / 2,
            'slider_back': -0.019 + (0.008 - -0.019) / 2,
            'slider_left': 0.015 + (0.041 - 0.015) / 2,
            'slider_right': -0.018 + (0.009 - -0.018) / 2,
            'top_prop_continuous': 0.0,  # Initial rotational velocity
            'bottom_prop_continuous': 0.0,  # Initial rotational velocity
        }

        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Subscriber for teleop twist keyboard inputs
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info("Birotor Drone Controller Node Initialized")

    def cmd_vel_callback(self, msg):
        # Parse the Twist message
        linear = msg.linear
        angular = msg.angular

        if linear.z > 0:  # Ascend
            self.set_sliders_max()
            self.set_propellers_spin(rate=1.0)  # Set propellers to max spin
        elif linear.z < 0:  # Descend
            self.set_sliders_min()
            self.set_propellers_spin(rate=0.5)  # Set propellers to a slower spin
        elif linear.x > 0:  # Move forward
            self.move_forward()
            self.set_propellers_spin(rate=0.8)
        elif linear.x < 0:  # Move backward
            self.move_backward()
            self.set_propellers_spin(rate=0.8)
        elif angular.z > 0:  # Move left
            self.move_left()
            self.set_propellers_spin(rate=0.8)
        elif angular.z < 0:  # Move right
            self.move_right()
            self.set_propellers_spin(rate=0.8)
        else:  # Hover
            self.set_sliders_mid()
            self.set_propellers_spin(rate=0.7)  # Maintain mid-level spin for hover

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
        self.joint_states['top_prop_continuous'] = rate
        self.joint_states['bottom_prop_continuous'] = rate

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = list(self.joint_states.keys())
        joint_state_msg.position = list(self.joint_states.values())

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
