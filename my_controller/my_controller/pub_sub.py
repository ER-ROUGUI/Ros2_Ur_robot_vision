import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
import time
import numpy as np
def tableDHM(q):
  
    tableDHM_out = np.array([ 
                            [q[0]  ,  0,       0.15185 ,    np.pi/2],
                            [q[1]  , -0.24355 ,  0      ,    0],
                            [q[2]  , -0.2132,  0      ,    0],
                            [q[3]  ,  0,       0.13105 ,   np.pi/2],
                            [q[4]  ,  0 ,      0.08535 ,   -np.pi/2],
                            [q[5]  ,  0 ,      0.0921  ,   0]]) # Same DHM parameters as Q2
                           # ADD T OOLS DHM PARAMETERS HERE
    

    return tableDHM_out

def gi_i1(theta_i, d_i, r_i,alpha_i ):
    


    gi_i1_out=np.array([[np.cos(theta_i),                   -np.sin(theta_i)*np.cos(alpha_i)    , np.sin(theta_i)*np.cos(alpha_i) ,   d_i*np.cos(theta_i)],
                        [np.sin(theta_i) ,                   np.cos(alpha_i)*np.cos(theta_i) ,  -(np.cos(theta_i))*np.sin(alpha_i) ,   d_i*np.sin(theta_i)],
                        [0 ,                                  np.sin(alpha_i) ,                  np.cos(alpha_i) ,                                    r_i],
                        [0,                                   0,                                 0,                                                    1]])
                 

    return gi_i1_out



def list_gi_i1(q_cur):

    tableDHM_q_cur = tableDHM(q_cur)
    Nb_rows_TableDHM = tableDHM_q_cur.shape[0]
    list_g_i_1_i = []

        
    for i in range(Nb_rows_TableDHM):
        g_i_1_i = gi_i1(tableDHM_q_cur[i][0], tableDHM_q_cur[i][1], tableDHM_q_cur[i][2], tableDHM_q_cur[i][3])
        list_g_i_1_i.append(g_i_1_i)
            

    return list_g_i_1_i

def list_g_0i(q_cur):
    
    list_gi_i1_out = list_gi_i1(q_cur)
    
    tableDHM_q_cur = tableDHM(q_cur)
    Nb_rows_TableDHM = tableDHM_q_cur.shape[0]
    
    list_g_0i_out=[]
    list_g_0i_out.append(list_gi_i1_out[0])
    for i in range(1,Nb_rows_TableDHM):
        list_g_0i_out.append(list_g_0i_out[-1].dot(list_gi_i1_out[i]))
        
    return list_g_0i_out

qd =np.array([0, -1.74532925 , 0 ,  -1.74532925 , 0  ,   0])
# print(list_g_0i(qd))
g0E = list_g_0i(qd)
#print(g0E[5])

############################################################" kinematic model ur3e "
numberJoints = 6
jointsType = ['R','R','R','R','R','R']
def ComputeJac(q_cur):
   
  
   # Inputs
    global numberJoints, jointsType # Input variables that come from outside this function
    list_g_0i_out = list_g_0i(q_cur)
    Zi=[0,0,1]
    P0N=list_g_0i_out[-1][0:3,3]
    J=np.zeros((6,6))
    for i in range(numberJoints):
        ROi= list_g_0i_out[i][0:3,0:3]
        POi =list_g_0i_out[i][0:3,3]
        Ji= np.cross(ROi.dot(Zi),P0N-POi)
        J[:,i] = np.concatenate((Ji,ROi.dot(Zi)), axis=0)
       
        
    return J
class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('pub_sub')
        self.joint_positions = [] 
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def joint_state_callback(self, msg):
        joint_positions = msg.position
        self.joint_positions = joint_positions 
        # self.get_logger().info(f"Received joint positions: {joint_positions}")

    def get_joint_positions(self):
        return self.joint_positions

class JointVelocityPublisher(Node):
    def __init__(self, controller_name='forward_velocity_controller' ):
        super().__init__('pub_sub')
        publish_topic = f"/{controller_name}/commands"  # Custom publish topic
        self.publisher = self.create_publisher(Float64MultiArray, publish_topic, 10)
        self.timer = self.create_timer(2, self.timer_callback)
        self.i = 0
       
        self.joint_state_subscriber = JointStateSubscriber()

    def timer_callback(self):
        current_joint_positions = self.joint_state_subscriber.get_joint_positions()
    
        # Print received joint positions and their length
        self.get_logger().info(f"Received joint positions: {current_joint_positions}")
        self.get_logger().info(f"Number of joints: {len(current_joint_positions)}")

        if not current_joint_positions or len(current_joint_positions) < 6:
            self.get_logger().warn("Invalid joint positions. Skipping publisher update.")
            return

        # Add your logic for computing joint velocities using the Jacobian here

        # msg = Float64MultiArray(data=joint_velocities)
        # self.publisher.publish(msg)
        # self.get_logger().info(f"Published joint velocities: {joint_velocities}")
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    joint_velocity_publisher = JointVelocityPublisher()
    joint_state_subscriber = JointStateSubscriber()

    try:
        while rclpy.ok():
            rclpy.spin_once(joint_velocity_publisher)
            rclpy.spin_once(joint_state_subscriber)
    except KeyboardInterrupt:
        pass

    joint_velocity_publisher.destroy_node()
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
