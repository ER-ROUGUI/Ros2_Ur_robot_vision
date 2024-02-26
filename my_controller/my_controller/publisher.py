#  Any question? you can send E-mail to saad.errougui@hotmail.com
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
# from image_detection import CircleDetector
from my_controller.image_detection_qr import QRDetector

# from subcriber import JointStateSubscriber
import math
import time
import numpy as np
def tableDHM(q):  # DH Parametres and not MDH !!!!
  
    tableDHM_out = np.array([ 
                            [q[0]          ,  0        ,       0.15185 ,    np.pi/2 ],
                            [q[1]          , -0.24355  ,       0       ,    0       ],
                            [q[2]          , -0.2132   ,       0       ,    0       ],
                            [q[3]          ,  0        ,       0.13105 ,    np.pi/2 ],
                            [q[4]          ,  0        ,       0.08535 ,   -np.pi/2 ],
                            [q[5]          ,  0        ,       0.0921  ,    0       ],
                            [  0           ,  0        ,       0       ,    0.001   ]]) # The End-effector Transformation
                           # ADD T OOLS DHM PARAMETERS HERE
    

    return tableDHM_out

def transformation_i_i_1(theta_i, r_i, d_i,alpha_i ):
    


    transformation_i_i_1_out = np.array([[np.cos(theta_i),     -np.sin(theta_i)*np.cos(alpha_i)    , np.sin(theta_i)*np.sin(alpha_i) ,   r_i*np.cos(theta_i)],
                                        [np.sin(theta_i) ,     np.cos(theta_i)*np.cos(alpha_i) ,  -(np.cos(theta_i))*np.sin(alpha_i) ,   r_i*np.sin(theta_i)],
                                        [0 ,                   np.sin(alpha_i) ,                  np.cos(alpha_i) ,                                    d_i],
                                        [0,                            0,                                 0,                                              1]])
                 


    return transformation_i_i_1_out



def list_gi_i1(q_cur):

    tableDHM_q_cur = tableDHM(q_cur)
    Nb_rows_TableDHM = tableDHM_q_cur.shape[0]
    list_g_i_1_i = []

        
    for i in range(Nb_rows_TableDHM):
        g_i_1_i = transformation_i_i_1(tableDHM_q_cur[i][0], tableDHM_q_cur[i][1], tableDHM_q_cur[i][2], tableDHM_q_cur[i][3])
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

# For testing
#qd =np.array([0, -np.pi/2 , -np.pi/2 , -np.pi/2 , np.pi/2 + np.pi ,   -np.pi/2])
## print(list_g_0i(qd))
#g0E = list_g_0i(qd)
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

def endeffector_camera():
    # Rotation matrix around x-axis by 30 degrees pi/6 rad // camera wrt End Effector
    transformation_matrix_Camera = np.array([
                                    [1   , 0               , 0              ,  0    ],
                                    [0   , np.cos(np.pi/6) , -np.sin(np.pi/6),  0    ],
                                    [0   , np.sin(np.pi/6), np.cos(np.pi/6),  -0.15],
                                    [0   , 0               , 0              ,  1    ]
                                                                                    ])



    return transformation_matrix_Camera

def skew_matrix(vector):
    if vector.shape == (3, ) :
        return np.array([
            [0, -vector[2], vector[1]],
            [vector[2], 0, -vector[0]],
            [-vector[1], vector[0], 0]
        ])
    else:
        raise ValueError("Input vector must be a 3x1 or 1x3 numpy array.")

def interaction_matrix(normalised_cordinates, depth):

    number_point = len(normalised_cordinates)
    Ls = []  # Initialize an empty list

    for i in range(0, number_point):
        x= normalised_cordinates[i][0]
        y= normalised_cordinates[i][1]
        z = depth[i]

        row1 = [-1/z, 0, x/z, x*y, -(1+x**2), y]
        row2 = [0, -1/z, y/z, 1+y**2, -x*y, -x]

        # Append rows to the list
        Ls.append(row1)
        Ls.append(row2)

    return np.array(Ls)


def inv_interaction(matrix):
    try:
        inverse_matrix = np.linalg.inv(matrix)
        return inverse_matrix
    except np.linalg.LinAlgError:
        pseudo_inverse_matrix = np.linalg.pinv(matrix)
        return pseudo_inverse_matrix
    
class JointVelocityPublisher(Node):
    def __init__(self, controller_name='forward_velocity_controller' ):
        super().__init__('publisher')
        publish_topic = f"/{controller_name}/commands"  # Custom publish topic
        self.publisher = self.create_publisher(Float64MultiArray, publish_topic, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.last_received_msg = None
        self.i = 0

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.image_constructor = QRDetector()
        self.fixed_points = [
                    (100, 100),
                    (500, 100),
                    (500, 400),
                    (100,400)
                ]
        
        # self.image_constructor.display_camera_stream()
        

        self.qr_coordinates = self.image_constructor.detect_qr_and_display(self.image_constructor.get_image_from_camera())
        self.normalized_cordinates = [self.image_constructor.normalized_coordinates(Qr_corner[0], Qr_corner[1]) for Qr_corner in self.qr_coordinates]
        
        qr_coordinates = self.image_constructor.detect_qr_and_display(self.image_constructor.get_image_from_camera())
        normalized_cordinates = [self.image_constructor.normalized_coordinates(Qr_corner[0], Qr_corner[1]) for Qr_corner in qr_coordinates]
        
        
        self.depth=[0,8, 0.8, 0.8 ,0.8]
        global interaction_matrix_result
        interaction_matrix_result = interaction_matrix(self.normalized_cordinates, self.depth)
        # error = self.image_constructor.calculate_error(
        #     self.image_constructor.get_image_from_camera(),
        #     fixed_points,
        #     circles
        # )
       
    
    
    def joint_state_callback(self, msg):

        joint_positions = np.array((msg.position))
        # print(np.shape(joint_positions))
        rearranged_positions = [joint_positions[5], joint_positions[0], joint_positions[1], 
                                joint_positions[2], joint_positions[3], joint_positions[4]]
        # joint_positions = msg.position
        self.joint_positions = rearranged_positions
        # self.get_logger().info(f"Received joint positions: {joint_positions}")
        # print((joint_positions))
        self.last_received_msg = msg 
        return rearranged_positions

        # return
        # self.joint_state_subscriber = JointStateSubscriber()

    def timer_callback(self):
        current_joint_positions = self.joint_state_callback(self.last_received_msg)
        # current_joint_positions=np.array([0,-np.pi/2,np.pi/4,-np.pi/2 ,-np.pi/2 ,0])
        lamda =0.2
        jacobian_matrix = ComputeJac(current_joint_positions)
        Jacob_inverse = np.linalg.pinv(jacobian_matrix)
        

        G0_E = list_g_0i(current_joint_positions)[5]
        End_effector_Base_Rot =  G0_E[:3, :3]
        zero_matrix = np.zeros((3, 3))

        End_effector_Base_Transformation = np.block([[End_effector_Base_Rot, np.zeros_like(End_effector_Base_Rot)],
                                                      [np.zeros_like(End_effector_Base_Rot),
                                                        End_effector_Base_Rot]])
        
        Camera_end_Effector = np.block([[endeffector_camera()[:3, :3], np.dot(skew_matrix(endeffector_camera()[:3, 3]),endeffector_camera()[:3, :3])],
                                         [np.zeros_like(End_effector_Base_Rot), endeffector_camera()[:3, :3]]])

        error = self.image_constructor.calculate_error(
            self.image_constructor.get_image_from_camera(),
            self.fixed_points,
            self.qr_coordinates
        )

        iterations= 80
        for iter in iterations:

            joint_velocities  = -lamda * np.dot(np.dot(np.dot(np.dot(Jacob_inverse,End_effector_Base_Transformation),
                                                Camera_end_Effector),inv_interaction(interaction_matrix_result)),error)

            # end_effector_velocity = np.array([0.0, 0.0, 0.0000001, 0.0, 0.0, 0.0])
            # joint_velocities = np.linalg.pinv(jacobian_matrix).dot(end_effector_velocity)
            # print(joint_velocities)
            
            # msg = Float64MultiArray(data=joint_velocities)
            # self.publisher.publish(msg)
            print(joint_velocities)
            # self.get_logger().info(f"Published joint velocities: {joint_velocities}")
            # self.get_logger().info(f"position values: {current_joint_positions}")
            self.i += 1


def main(args=None):
    rclpy.init(args=args)
    controller_name = 'forward_velocity_controller'  # Replace with your desired controller name
    joint_velocity_publisher = JointVelocityPublisher(controller_name)
    rclpy.spin(joint_velocity_publisher)
    # joint_velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
