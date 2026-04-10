import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import ApplyJointEffort
from builtin_interfaces.msg import Duration, Time
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Empty

class JointEffortController(Node):
    def __init__(self):
        super().__init__('joint_effort_controller')

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.effort_client = self.create_client(ApplyJointEffort, '/apply_joint_effort')
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.unpause_client = self.create_client(Empty, '/unpause_physics')

        # Service client to apply effort in Gazebo
        self.effort_client = self.create_client(ApplyJointEffort, '/apply_joint_effort')
        self.joint_positions = {}
        # Run startup sequence
        self.startup_sequence()


    def startup_sequence(self):
        # 1. Unpause physics
        self.get_logger().info('Unpausing physics...')
        self.unpause_client.wait_for_service(timeout_sec=5.0)
        future = self.unpause_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Physics unpaused')

        # 2. Deactivate trajectory controller
        self.get_logger().info('Deactivating trajectory controller...')
        self.switch_client.wait_for_service(timeout_sec=5.0)
        req = SwitchController.Request()
        req.activate_controllers = []
        req.deactivate_controllers = ['joint_trajectory_controller']
        req.strictness = 1
        future = self.switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Trajectory controller deactivated — ready to send efforts')

    def joint_state_callback(self, msg):
        # Store latest joint positions by name
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

        pos = self.joint_positions.get('joint3', None)
        if pos is not None:
            self.get_logger().info(f'joint3 position: {pos:.4f}')
            self.apply_effort('joint3', effort= 5000.0)

    def apply_effort(self, joint_name, effort):
        if not self.effort_client.service_is_ready():
            self.get_logger().warn('Service not ready')
            return

        req = ApplyJointEffort.Request()
        req.joint_name = joint_name
        req.effort = effort
        req.start_time = Time(sec=0, nanosec=0)
        req.duration = Duration(sec=50, nanosec=100_000_000)

        # Async call — doesn't block the subscriber
        self.effort_client.call_async(req)


def main():
    rclpy.init()
    node = JointEffortController()
    rclpy.spin(node)
    rclpy.shutdown()
