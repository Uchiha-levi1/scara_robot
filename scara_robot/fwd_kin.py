import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import math
import numpy as np
from numpy import cos,sin,pi,deg2rad

class fwd_kin(Node):
    def __init__(self):
        super().__init__('fwd_kin_node') #naming my node 
        self.sub=self.create_subscription(JointState, '/joint_states',self.listener_callback,10)#listens to joints
        self.pub=self.create_publisher(Pose,'/ee_pose',10) #publishes the end effector pose
        self.get_logger().info('Node started. Listening to /joint_states...')
    @staticmethod
    def tranf(t,d,a,alp):
        T=[[cos(t),-sin(t)*cos(alp),sin(t)*sin(alp),a*cos(t)],
            [sin(t),cos(t)*cos(alp),-cos(t)*sin(alp),a*sin(t)],
            [0     ,  sin(alp)      ,  cos(alp)      , d],
            [0     ,     0          ,    0            ,1]]
        return np.array(T)
    

    
    def listener_callback(self, msg):

        joints = {}
        for i, name in enumerate(msg.name):
            joints[name]=msg.position[i]

        q1=joints['joint1'] 
        q2=joints['joint2']
        q3=joints['joint3'] #  the prismatic joint

        no_joints=5 # 3 joints, 2 transformations for J1 and EE postion transformations
        #no_frames=no_joints+2
        d=np.array([1.5,0,0.1,q3,-0.15] )#defining all my DH parameters
        a=np.array([0,0.8,0.6,0,0])
        alp=deg2rad(np.array([0,0,0,0,0]))
        theta=np.array([0,q1,q2,0,0])
        
        T_all=np.eye(4) #initialising with T01 which is identity

        for i in range(no_joints): #post multiplying each tranformation
            T=self.tranf((theta[i]),d[i],a[i],alp[i])
            T_all=T_all@T

        p=Pose()
        p.position.x=T_all[0, 3] # extracting last column pose info
        p.position.y=T_all[1, 3]
        p.position.z=T_all[2, 3]
        r00,r01,r02,r10,r11,r12,r20,r21,r22=T_all[0:3,0:3].flatten()

        w=0.5*math.sqrt(1+r00+r11+r22) #formula to covert R matrix to quaternions
        p.orientation.w=w
        p.orientation.x=(r21-r12)/(4*w)
        p.orientation.y=(r02-r20)/(4*w)
        p.orientation.z=(r10-r01)/(4*w)

        self.pub.publish(p) # publishing the pose from received joint positions
        self.get_logger().info(f'EE Pose:\n' f'  X: {p.position.x:.4f}\n'f'  Y: {p.position.y:.4f}\n' f'  Z: {p.position.z:.4f}\n'
    f'  qw: {p.orientation.w:.4f}\n' f'  qx: {p.orientation.x:.4f}\n' f'  qy: {p.orientation.y:.4f}\n' f'  qz: {p.orientation.z:.4f}')
        

def main():
    rclpy.init()
    node = fwd_kin()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()