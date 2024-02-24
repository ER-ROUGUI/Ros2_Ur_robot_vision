from subcriber import JointStateSubscriber
import rclpy
import numpy as np
import time
def main():
    rclpy.init()

    joint_state_subscriber = JointStateSubscriber()

    try:
        while rclpy.ok():
            rclpy.spin_once(joint_state_subscriber)
            joint_positions = joint_state_subscriber.get_joint_positions()
            
            if joint_positions:
                reshaped_positions = np.array(joint_positions).reshape(-1, 1)
                flattened_positions = reshaped_positions.flatten()
                print("Transposed Positions:", flattened_positions)
                
    except KeyboardInterrupt:
        pass

    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()