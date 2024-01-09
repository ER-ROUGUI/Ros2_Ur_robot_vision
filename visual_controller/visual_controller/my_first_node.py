import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PublisherJointTrajectory(Node):
    def __init__(self):
        self.i=0
        super().__init__("my_first_node")

        # Declare ROS parameters
        self.declare_parameter("controller_name", "joint_trajectory_controller") # other controller     joint_trajectory_controller ,scaled_joint_trajectory_controller
        self.declare_parameter("wait_sec_between_publish", 6.0)

        # Read ROS parameters
        controller_name = self.get_parameter("controller_name").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value

        # Convert goal names and positions to floats
        # Goal positions are now defined directly in the code (without the goal_names parameter)
        # goal_positions = [
        #     [0.0, -1.57, 0.0, -1.57,0.0 ,0.0],
        #     [0.0, 0.0, -1.0 ,0.1,0.0,0.2]
        # ]
		
        # Create and publish joint trajectory
        publish_topic = "/" + controller_name + "/" + "joint_trajectory"
        self.get_logger().info(f"Publishing joint trajectories on topic '{publish_topic}' every {wait_sec_between_publish} s")
        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
		
    goal_index = 0
    def timer_callback(self):

        
        goal_positions = [
            [0.785, -1.57, 0.785, 0.785, 0.785, 0.785],
            [0.0, -1.57, 0.0, 0.0, 0.0, 0.0],
            [0.0, -1.57, 0.0, 0.0, -0.785, 0.0],
            [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]

        ]

        # Create a joint trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        # Create a point in the joint trajectory
        point = JointTrajectoryPoint()

        # Set the joint positions for the point
        
        point.positions = [float(joint)
            for joint in goal_positions[self.i % len(goal_positions)]
        ]

        # Set the time from start for the point
        point.time_from_start = Duration(sec=4)
        print(point)

        # Append the point to the joint trajectory
        trajectory.points.append(point)

        # Publish the joint trajectory
        self.publisher_.publish(trajectory)

        # Update the index of the goal position
        self.i += 1


if __name__ == "__main__":
    rclpy.init()

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


####################################


# import rclpy
# from rclpy.node import Node
# from builtin_interfaces.msg import Duration

# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from transformation_matrix import *
# from ur3e_kinematics import UR3eKinematics

# class PublisherJointTrajectory(Node):
#     def __init__(self):
#         self.i=0
#         super().__init__("my_first_node")

#         # Declare ROS parameters
#         self.declare_parameter("controller_name", "joint_trajectory_controller") # other controller     joint_trajectory_controller ,scaled_joint_trajectory_controller
#         self.declare_parameter("wait_sec_between_publish", 6.0)

#         # UR3e kinematics solver
#         self.kinematics_solver = UR3eKinematics()

#         # Give Cartesian poses from user input
#         joint_angles = []
#         while True:
#             pose = input("Enter a Cartesian pose (x y z) (e.g., 0.0 0.0 0.1): ")
#             if pose == "exit":
#                 break
#             pose = [float(x) for x in pose.split()]
#             joint_angles.append(self.kinematics_solver.inverse_kinematics(pose))

#         # Convert Cartesian positions to joint positions
#         joint_positions = []
#         for pose in joint_angles:
#             joint_positions.append([float(joint) for joint in pose])

#         # Create and publish joint trajectory
#         publish_topic = "/" + controller_name + "/" + "joint_trajectory"
#         self.get_logger().info(f"Publishing joint trajectories on topic '{publish_topic}' every {wait_sec_between_publish} s")
#         self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
#         self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)

#     goal_index = 0
#     def timer_callback(self):

#         # Select a Cartesian pose from the list of poses
#         pose = joint_positions[self.i % len(joint_positions)]

#         # Convert Cartesian pose to joint angles
#         joint_angles = pose

#         # Create a joint trajectory message
#         trajectory = JointTrajectory()
#         trajectory.joint_names = [
#             "shoulder_pan_joint",
#             "shoulder_lift_joint",
#             "elbow_joint",
#             "wrist_1_joint",
#             "wrist_2_joint",
#             "wrist_3_joint"
#         ]

#         # Create a point in the joint trajectory
#         point = JointTrajectoryPoint()

#         # Set the joint positions for the point
#         point.positions = pose

#         # Set the time from start for the point
#         point.time_from_start = Duration(sec=4)
#         print(point)

#         # Append the point to the joint trajectory
#         trajectory.points.append(point)

#         # Publish the joint trajectory
#         self.publisher_.publish(trajectory)

#         # Update the index of the Cartesian pose
#         self.i += 1


# if __name__ == '__main__':
#     rclpy.init()
#     node = PublisherJointTrajectory()
#     rclpy.spin(node)
#     rclpy.shutdown()