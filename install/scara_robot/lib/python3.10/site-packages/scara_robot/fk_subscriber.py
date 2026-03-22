import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose 


def dh_transform(a, alpha, d, theta):
    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,     sa,      ca,      d],
        [0.0,    0.0,     0.0,    1.0],
    ])


class ForwardKinematicsSubscriber(Node):

    def __init__(self):
        super().__init__('fk_subscriber')

        # Listening to Gazebo
        self.subscription = self.create_subscription(
            JointState,            
            '/joint_states',       # ← Gazebo publishes here
            self.joint_callback,
            10
        )
        
        # publishes pose as a topic 
        self.publisher_ = self.create_publisher(
            Pose,
            '/end_effector_pose',
            10
        )

        self.DH_PARAMS = [
            #(a,    alpha,  d,  theta_offset) assigned random values
            (1.2,   0.0,    0.5, 0.0),      #Joint 1 - Revolute (a1 = link 1 = 1.2)
            (1.3,   0.0,    0.0, 0.0),      #Joint 2 - Revolute (a2 = link 2 = 1,3)
            (0.0,  math.pi, 0.0, 0.0),     #Joint 3 - Prismatic (d3 varies from 0.0 to 3.0m)
        ]
        
       	self.JOINT_TYPES = ['revolute','revolute','prismatic']
        self.get_logger().info("FK Subscriber Started")
        self.last_print_time = self.get_clock().now()

    def joint_callback(self, msg):
    
        # Only print every 5 seconds
        now = self.get_clock().now()
        elapsed = (now - self.last_print_time).nanoseconds / 1e9

        if elapsed < 5.0:
            return
    
        self.last_print_time = now
    
        q = list(msg.position)

        if len(q) != 3:
            self.get_logger().error("Expected 3 joint values.")
            return

        T = np.eye(4)

        for i in range(len(self.DH_PARAMS)):
            a, alpha, d, theta_offset = self.DH_PARAMS[i]
            
            if self.JOINT_TYPES[i] =='prismatic':
            	d =-q[i]     # q[3] is linear distance (0 to 3.0)and is moving opposite to z3
            	theta = theta_offset
            	
            else:
            	theta =q[i] + theta_offset  #revolute
            	
            T = T @ dh_transform(a, alpha, d, theta)
        
        position = T[0:3, 3]  # Extract position
        R = T[0:3, 0:3]       # Extract rotation
        
        # Convert rotation matrix to quaternion
        trace = R[0,0] + R[1,1] + R[2,2]
        
        if trace > 0:
    	    s = 0.5 / math.sqrt(trace + 1.0)
    	    w = 0.25 / s
    	    x = (R[2,1] - R[1,2]) * s
    	    y = (R[0,2] - R[2,0]) * s
    	    z = (R[1,0] - R[0,1]) * s
    	    
        else:
            w, x, y, z = 1.0, 0.0, 0.0, 0.0 
        
        # Build and publish Pose message
        pose_msg = Pose()
        pose_msg.position.x = float(position[0])
        pose_msg.position.y = float(position[1])
        pose_msg.position.z = float(position[2])
        pose_msg.orientation.x = float(x)
        pose_msg.orientation.y = float(y)
        pose_msg.orientation.z = float(z)
        pose_msg.orientation.w = float(w)
        
        self.publisher_.publish(pose_msg)
        
        self.get_logger().info(
            "\n=== Homogeneous Transformation Matrix ===\n"+
            np.array2string(T, precision=4, suppress_small=True)+
            "\n=========================================" +
            
            "\n=== End Effector Pose ===\n"
            f"Position: x={position[0]:.4f}, "
            f"y={position[1]:.4f}, "
            f"z={position[2]:.4f}\n"
            "========================="
        )


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
