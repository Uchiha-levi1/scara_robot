import rclpy
from rclpy.node import Node
from scara_interfaces.srv import IK
import numpy as np
import math
from numpy import cos, sin, sqrt, arctan2

class inv_kin(Node):
    def __init__(self):
        super().__init__('inv_kin_node')
        self.srv=self.create_service(IK,'inverse_kinematics',self.ik_callback)
        self.get_logger().info('IK srv starting up...')
        self.l1=0.8  # initializing the link lengths 
        self.l2=0.6  

    def ik_callback(self,req,resp):
        x=req.x# reading req witht the EE position
        y=req.y
        z=req.z
        #resp has to print joint1, joint2,joint3

        resp.joint3=z-1.45  # base_height(1.5)+j2 offset(0.1)+EEoffset from J3(-0.15)

        cos_q2 = (x**2+y**2-self.l1**2-self.l2**2) / (2*self.l1*self.l2)
        q2 = arctan2(sqrt(1-cos_q2**2),cos_q2)  # taking +ve soltion, solution in report
        cos_beta=(self.l1**2+x**2+y**2-self.l2**2)/(2*math.sqrt(x**2+y**2)*self.l1)
        alpha=arctan2(y,x)
        q1_and_alpha= arctan2(sqrt(1-cos_beta**2),cos_beta)#q1 and alpha=alpha+(-q1) check diagram for the way it is meansured
        q1=-(q1_and_alpha-alpha)
        resp.joint1=float(q1)
        resp.joint2=float(q2)
        resp.joint3=float(resp.joint3)
        self.get_logger().info(f'  joint1={np.degrees(q1):.2f} deg\n' f'  joint2={np.degrees(q2):.2f} deg\n' f'  joint3={resp.joint3:.4f} m')

        return resp

def main():
    rclpy.init()
    node=inv_kin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()