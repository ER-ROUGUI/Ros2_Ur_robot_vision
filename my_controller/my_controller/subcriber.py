import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time
class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('subcriber')
        self.joint_positions = [] 
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def joint_state_callback(self, msg):
        joint_positions = np.array(list(msg.position))
        # joint_positions = msg.position
        self.joint_positions = joint_positions 
        self.get_logger().info(f"Received joint positions: {joint_positions}")
        # print(type(joint_positions))
       
        # time.sleep(2)
        

    def get_joint_positions(self):
        
        return self.joint_positions

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()

    try:
        while rclpy.ok():
            # Faites quelque chose avec joint_state_subscriber.get_joint_positions()
            rclpy.spin_once(joint_state_subscriber)
    except KeyboardInterrupt:
        pass

    # rclpy.spin(joint_state_subscriber)
    joint_state_subscriber.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
