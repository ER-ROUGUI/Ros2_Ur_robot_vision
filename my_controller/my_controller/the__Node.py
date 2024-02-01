import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint 
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import cv2
import requests
import numpy as np
from ur3e_model import*
import numpy as np
import time

class CircleDetector:
    def __init__(self, camera_url= "http://192.168.1.102:4242/current.jpg?annotations=on/off", param1=50, param2=30, min_radius=10, max_radius=50, num_circles=4):
        self.camera_url = camera_url
        self.param1 = param1
        self.param2 = param2
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.num_circles = num_circles
        self.cap = cv2.VideoCapture(self.camera_url)

    
    def get_image_from_camera(self):
        try:
            response = requests.get(self.camera_url)
            img_array = np.array(bytearray(response.content), dtype=np.uint8)
            img = cv2.imdecode(img_array, -1)
            
            return img
        except Exception as e:
            #print(f"Erreur lors de la récupération de l'image: {e}")
            return None
        

    def detect_circles_and_display(self):

        img = self.get_image_from_camera()
        if img is not None:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            circles = cv2.HoughCircles(
                gray_blurred,
                cv2.HOUGH_GRADIENT,
                dp=1,
                minDist=20,
                param1=self.param1,
                param2=self.param2,
                minRadius=self.min_radius,
                maxRadius=self.max_radius
            )

            if circles is not None:
                circles = np.uint16(np.around(circles))[0, :]
                circles = sorted(circles, key=lambda x: x[2], reverse=True)[:self.num_circles]

                for i, circle in enumerate(circles):
                    center = (circle[0], circle[1])
                    radius = circle[2]

                    cv2.circle(img, center, radius, (0, 255, 0), 2)
                    #print(f"Position du cercle {i+1} - x: {center[0]}, y: {center[1]}, rayon: {radius}")

                return circles
            
            

            else:
                return []
        else:
            return []
        
    def display_camera_stream(self):
        while True:
            img = self.get_image_from_camera()
            if img is not None:
                circles = self.detect_circles_and_display()

                for circle in circles:
                    center = (circle[0], circle[1])
                    radius = circle[2]
                    cv2.circle(img, center, radius, (0, 255, 0), 2)

                # Afficher les points fixes
                fixed_points = [
                    (100, 100),
                    (300, 100),
                    (100, 400)
                ]
                for point in fixed_points:
                    cv2.circle(img, point, 5, (255, 0, 0), -1)

                # Afficher l'image
                cv2.imshow('Camera Stream', img)
            
            # Press 'q' to exit the stream
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Release the camera when the stream is closed
        self.cap.release()
        cv2.destroyAllWindows()
                
    # def release_camera(self):
    #     self.cap.release()

class PublisherJointTrajectory(Node):
    def __init__(self):
        self.i = 0  # Initialize self.i
        super().__init__("my_first_node")
        
        # Declare ROS parameters

        self.declare_parameter("camera_url", "http://192.168.1.102:4242/current.jpg?annotations=on/off")
        self.declare_parameter("controller_name", "forward_velocity_controller")
        self.declare_parameter("wait_sec_between_publish", 5.0)
        
        self.camera_url = self.get_parameter("camera_url").value
        self.timer = self.create_timer(1, self.timer_callback)
        controller_name = self.get_parameter("controller_name").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value

        

        publish_topic = "/" + controller_name + "/" + "commands"
        self.get_logger().info(f"Publishing joint velocities on topic '{publish_topic}' every {wait_sec_between_publish} s")
        self.publisher_ = self.create_publisher(Float64MultiArray, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        

        joint_state_topic = "/joint_states"
        self.get_logger().info(f"Subscribing to joint state topic '{joint_state_topic}'")
        self.joint_state_subscriber = self.create_subscription(JointState ,joint_state_topic,self.joint_state_callback,1)
        self.latest_joint_state = None


        self.circle_detector = CircleDetector()

    def get_camera_image(self):
        return self.circle_detector.get_image_from_camera()
    
    def joint_state_callback(self, msg):

        joint_positions = msg.position
        #self.get_logger().info(f"Received joint positions: {joint_positions}")
        self.latest_joint_state = joint_positions

        global q_cur
        # q_cur = list(joint_positions)
        #q_cur =np.array([-np.pi/2,0,-np.pi/2 ,-np.pi/2 ,-np.pi/2 ,-np.pi/2])
        
    def get_latest_joint_state(self):
        # Return the latest joint state vector
        return self.latest_joint_state
    
    def timer_callback(self):

        fixed_points = [
                    (100, 100),
                    (300, 100),
                    (100, 400)
                ]
     
        circles = self.circle_detector.detect_circles_and_display()

        for circle in circles:
            center = (circle[0], circle[1])
            radius = circle[2]
            cv2.circle(camera_image, center, radius, (0, 255, 0), 2)

        camera_image = self.get_camera_image()
        cv2.imshow("Camera Image", camera_image) 
        #print(circles)
        errors = calculate_errors(circles, fixed_points)
        #self.get_logger().info(f"Errors: {errors}")

        q_cur =np.array([-np.pi/2,0,-np.pi/2 ,-np.pi/2 ,-np.pi/2 ,-np.pi/2])

        

        #jac_result = ComputeJac(q_cur)
        Jacob_inverse = np.linalg.inv(ComputeJac(q_cur))
        #print(Jacob_inverse)
        # ikm_result = ComputeIKM(X_d_i, X_d_f, V, Te, q_i, k_max, eps_x)
        #ee_camera_result = endeffector_camera()
        #normalized_result = normalized_coordinates(x_pixel=500, y_pixel = 300, fx=100, fy=100, u0=30, v0=40)
        normalized_result = [[0.02, 0.03], [0.05, 0.01], [0.04, 0.03]]
        depth =[1,1,1]
        interaction_matrix_result = interaction_matrix(normalized_result , depth)

        # joint_velocities = [
        #     [0.0, 0.0, 0.0, 0.0, 0.08, 0.0],
        #     [0.0, 0.0, 0.0, 0.0, -0.08, 0.0],
        #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # ]
        lamda = 0.2
        G0_E = list_g_0i(q_cur)[5]
        End_effector_Base_Rot =  G0_E[:3, :3]
        zero_matrix = np.zeros((3, 3))
        #circles = [(100, 100, 10), (200, 200, 15), (300, 300, 20)]
        #fixed_points = [(90, 90), (190, 190), (290, 290)]
        current_error = calculate_errors(circles, fixed_points)
        #current_error = [[1,2],[1,2],[1,2], [1,2], [1,2],[1,2]]
        #print(current_error)
        Camera_end_Effector = np.block([[endeffector_camera()[:3, :3], np.dot(skew_matrix(endeffector_camera()[:3, 3]),endeffector_camera()[:3, :3])], [np.zeros_like(End_effector_Base_Rot), endeffector_camera()[:3, :3]]])
        End_effector_Base_Transformation = np.block([[End_effector_Base_Rot, np.zeros_like(End_effector_Base_Rot)], [np.zeros_like(End_effector_Base_Rot), End_effector_Base_Rot]])
        #print("tttddddddddddddddddddddd" , inv_interaction(interaction_matrix_result))
        ########################################################## loi de commande
        
        joint_velocities = -lamda * np.dot(np.dot(np.dot(np.dot(Jacob_inverse, End_effector_Base_Transformation), Camera_end_Effector), inv_interaction(interaction_matrix_result)), current_error)
        print("Calculated Joint Velocities:", joint_velocities)
        
        ##################################################################
        msg = Float64MultiArray()
        msg.data = [float(velocity) for velocity in joint_velocities[self.i % len(joint_velocities)]]

        #self.publisher_.publish(msg)

        self.i += 1


def main(args=None):
    rclpy.init()
    publisher_joint_trajectory = PublisherJointTrajectory()
    #time.sleep(1)
    final_joint_state = publisher_joint_trajectory.get_latest_joint_state()
    publisher_joint_trajectory.circle_detector.display_camera_stream()
    #print(f"Final Joint State: {final_joint_state}")
    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

