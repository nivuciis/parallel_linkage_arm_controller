import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class JacobianController(Node):
    def __init__(self):
        super().__init__('jacobian_controller')

        
        self.L1 = 0.14  #Upper Arm
        self.L2 = 0.16  #Forearm
        self.base_height = 0.10 # base height from ground

        # INitial States
        self.current_joints = np.array([0.0, 0.0, 0.0, 0.0])
        self.target_pos = np.array([0.2, 0.0, 0.15]) # Target (X, Y, Z)

        # Subscribers e Publishers
        self.sub_joints = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.pub_cmd = self.create_publisher(
            Float64MultiArray, '/uarm/command', 10)
        
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("JACOBIAN CONTROLLER NODE STARTED")

    def joint_callback(self, msg):
        if len(msg.position) >= 3:
            self.current_joints = np.array(msg.position[:4])

    
    def forward_kinematics(self, joints):
        
        q1 = joints[0]
        q2 = joints[1]
        q3 = joints[2] 
        
        r = self.L1 * math.cos(q2) + self.L2 * math.cos(q3)
        
        x = math.cos(q1) * r
        y = math.sin(q1) * r
        z = self.base_height + self.L1 * math.sin(q2) + self.L2 * math.sin(q3)
        
        return np.array([x, y, z])

    
    def compute_jacobian(self, joints):
        delta = 0.001 
        J = np.zeros((3, 3)) 
        
        pos_atual = self.forward_kinematics(joints)
        
        for i in range(3):
        
            joints_perturbed = joints.copy()
            joints_perturbed[i] += delta
            
            pos_new = self.forward_kinematics(joints_perturbed)
            
            coluna_J = (pos_new - pos_atual) / delta
            
            J[:, i] = coluna_J
            
        return J

    def control_loop(self):
    
        current_xyz = self.forward_kinematics(self.current_joints)
        
        error = self.target_pos - current_xyz
        
        if np.linalg.norm(error) < 0.005:
            return

        J = self.compute_jacobian(self.current_joints)
        
        try:
            J_pinv = np.linalg.pinv(J)
        except np.linalg.LinAlgError:
            self.get_logger().warn("Singularity")
            return

        Kp = 5.0
        
        v_cartesian = error * Kp
        
        q_dot = np.dot(J_pinv, v_cartesian)
        
        dt = 0.02
        new_joints = self.current_joints.copy()
        
        new_joints[0] += q_dot[0] * dt
        new_joints[1] += q_dot[1] * dt
        new_joints[2] += q_dot[2] * dt
        

        new_joints[3] = 0.0 

        msg = Float64MultiArray()
        msg.data = new_joints.tolist()
        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JacobianController()
    
    node.target_pos = np.array([0.15, 0.05, 0.20]) 
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()