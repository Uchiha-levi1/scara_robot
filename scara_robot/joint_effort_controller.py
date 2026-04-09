import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import ApplyJointEffort
from builtin_interfaces.msg import Duration

class JointEffortController(Node):
    def __init__(self):
        super().__init__('joint_effort_controller')

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Service client to apply effort in Gazebo
        self.effort_client = self.create_client(ApplyJointEffort, '/apply_joint_effort')
        self.joint_positions = {}

    def joint_state_callback(self, msg):
        # Store latest joint positions by name
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    def apply_effort(self, joint_name, effort):
        if not self.effort_client.service_is_ready():
            self.get_logger().warn('Service not ready')
            return

        req = ApplyJointEffort.Request()
        req.joint_name = joint_name
        req.effort = effort
        req.start_time = Duration(sec=0, nanosec=0)  # immediate
        req.duration = Duration(sec=0, nanosec=100_000_000)  # 100ms

        # Async call — doesn't block the subscriber
        self.effort_client.call_async(req)


def main():
    rclpy.init()
    node = JointEffortController()
    rclpy.spin(node)
    rclpy.shutdown()
