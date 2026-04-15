import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import ApplyJointEffort
from builtin_interfaces.msg import Duration, Time
from controller_manager_msgs.srv import SwitchController
import subprocess
import os
from scara_interfaces.srv import SetRef

class JointEffortController(Node):
    def __init__(self):
        super().__init__('joint_effort_controller')

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.effort_client = self.create_client(ApplyJointEffort, '/apply_joint_effort')
        self.srv = self.create_service(SetRef,'set_reference',self.set_ref_cb) # custom service- inside scara_interfaces.srv
        self.joint_positions = {}
        

        self.prev_error=0.0
        self.prev_pos = -2.5

        self.prev_time=self.get_clock().now().nanoseconds * 1e-9
        self.log_file = open('joint_log.txt','w')
        self.kp=500 # porportional gain
        self.kd=45 # derivative gain
        self.ref=-1 # setting reference value to match 

    
    
    def set_ref_cb(self, request, response):
        self.ref=request.position # takes position and stores it for the node to process
        response.success = True #
        self.get_logger().info(f'New reference: {request.position}')
        return response


    def joint_state_callback(self, msg):
        # Store latest joint positions by name
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

        pos = self.joint_positions.get('joint3', None)
        if pos is not None:
            
            now=self.get_clock().now().nanoseconds * 1e-9 # take current time 
            dt=now- self.prev_time 

            error = self.ref-pos #finding error derivative
            if dt>0:
                error_dot=(error-self.prev_error) /dt
            else:
                error_dot = 0.0
            effort=self.kp*error + self.kd*error_dot
            
            self.prev_error = error
            self.prev_time = now
            t = self.get_clock().now().nanoseconds*1e-9 # getting time to visualize ing matlab
            self.log_file.write(f'{t},{self.ref},{pos}\n') # log the reference and current position at that instance
            
            
            self.apply_effort('joint3', effort)
            self.get_logger().info(f'joint3 position: {pos:.4f}')    
    def apply_effort(self, joint_name, effort):
        env = os.environ.copy()
        env['GAZEBO_MASTER_URI'] = 'http://localhost:11345'
        subprocess.Popen(
            ['/usr/bin/gz', 'joint', '-m', 'scara_robot', '-j', joint_name, '-f', str(effort)],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )       
"""
    def apply_effort(self, joint_name, effort):
        if not self.effort_client.service_is_ready():
            self.get_logger().warn('Service not ready')
            return

        req = ApplyJointEffort.Request()
        req.joint_name = joint_name
        req.effort = effort
        req.start_time = Time(sec=0, nanosec=0)
        req.duration = Duration(sec=0, nanosec=10_000_000)


        # Async call — doesn't block the subscriber
        self.effort_client.call_async(req)
        """
    

def main():
    rclpy.init()
    node = JointEffortController()
    rclpy.spin(node)
    rclpy.shutdown()
