import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from image_detection import CircleDetector  # Assuming you have a separate file for CircleDetector class
from ur3e_model import ComputeJac, endeffector_camera, list_g_0i, skew_matrix, interaction_matrix, inv_interaction
# from test import*
from subcriber import JointStateSubscriber

class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__("my_first_node")
        self.i = 0


        self.circle_detector = CircleDetector()
        joint_state_subscriber = JointStateSubscriber()
        self.circle_detector.display_camera_stream()

    def joint_state_callback(self, msg):
        joint_positions = msg.position
        self.latest_joint_state = joint_positions

    def timer_callback(self):
       
        
        current_error = self.circle_detector.calculate_error
        depth =[1,1,1]
        normalized_result = self.circle_detector.normalized_coordinates
        interaction_matrix_result = interaction_matrix(normalized_result , depth)


        G0_E = list_g_0i(q_cur)[5]
        End_effector_Base_Rot =  G0_E[:3, :3]
        lamda=0.2

        q_cur = self.joint_state_subscriber.get_joint_positions()
        Jacob_inverse = np.linalg.inv(ComputeJac(q_cur))
        Camera_end_Effector = np.block([[endeffector_camera()[:3, :3], np.dot(skew_matrix(endeffector_camera()[:3, 3]), endeffector_camera()[:3, :3])], [np.zeros_like(End_effector_Base_Rot), endeffector_camera()[:3, :3]]])
        End_effector_Base_Transformation = np.block([[End_effector_Base_Rot, np.zeros_like(End_effector_Base_Rot)], [np.zeros_like(End_effector_Base_Rot), End_effector_Base_Rot]])

        joint_velocities = -lamda * np.dot(np.dot(np.dot(np.dot(Jacob_inverse, End_effector_Base_Transformation), Camera_end_Effector), inv_interaction(interaction_matrix_result)), current_error)
        print("Calculated Joint Velocities:", joint_velocities)
        self.get_logger().info(f"Received joint positions: {joint_velocities}")

        msg = Float64MultiArray()
        # msg.data = [float(velocity) for velocity in joint_velocities[self.i % len(joint_velocities)]]

        # Publish joint velocities
        # self.publisher_.publish(msg)

        self.i += 1

    def get_latest_joint_state(self):
        return self.latest_joint_state


def main(args=None):
    rclpy.init()
    publisher_joint_trajectory = PublisherJointTrajectory()
    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
