
This project involves controlling a Universal Robot UR3e using ROS2 with the Humble distribution. It includes creating geometric and kinematic models of the robot 
based on its [DH parameters](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/), implementing Image-Based Visual Servoing (IBVS), and explaining the installation and configuration steps
## Installation Steps
### Installation ROS2
To begin, you'll need to install Ubuntu (version 22.04). Next, follow the steps outlined in the [official ROS2 Humble installation tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

Run the following command to add the line to your `~/.bashrc` file:




```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Installing Colcon for Building Packages

To build ROS 2 packages, we need to install Colcon. Follow these steps:

Run the following command to install Colcon:

```bash
sudo apt install python3-colcon-common-extensions python3-vcstool
```

Enable Colcon autocompletion by adding the following line to your `~/.bashrc` file:

```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```

Apply the changes by running:

```bash
source ~/.bashrc
```

Now, you have Colcon installed and autocompletion enabled for a smoother package-building.


## Installing the UR Driver for ROS 2 Humble

 To install the **ur_robot_driver** you can choose to either use the binary packages or install from source. To install using binary packages, use the following command:

```bash
sudo apt-get install ros-humble-ur
```

This will install the UR Robot Driver for ROS 2 Humble using binary packages. Adjust the installation method based on your preferences and requirements.

## Installing UR Driver and Configuring a UR Robot

Follow these steps to install the UR Driver and set up a UR robot for communication with ROS 2. Detailed instructions can be found [here](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/installation/robot_setup.html).

1. Install the UR Driver on the robot's polyscope using the provided [installation guide](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/installation/robot_setup.html).

2. Once the UR Driver is installed on the robot, you can test the communication with ROS 2 using the following command:

    ```bash
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<ur_type> robot_ip:=192.168.1.101 launch_rviz:=true
    ```

    Ensure to replace `<ur_type>` with the specific type of your Universal Robots robot ( ur3e , ur5e etc ) and set the correct robot IP address (e.g., `192.168.1.101`). This command launches the UR robot driver and initializes communication with ROS 2.

3. Optionally, visualize the state of the robot in Rviz. setting up launch_rviz parametre on True.

4. Run the **External Control** program from the PolyScope interface.
Now, your Universal Robots robot is ready to accept external control commands. Ensure that the IP address and port configuration match the parameters used in the ROS 2 launch command.

## Testing Communication with the Robot

To verify the communication between ROS 2 and the Universal Robots robot, follow these steps:

1. Open a new terminal and use the following command to echo the `/joint_states` topic, displaying the actual joint positions of the robot:

    ```bash
    ros2 topic echo /joint_states
    ```

   Observe the joint position values and ensure they reflect the current state of the robot.

2. Move the robot using the PolyScope interface on the robot controller. You should observe changes in the joint positions reflected in the terminal where you are echoing the `/joint_states` topic.
3. You can experiment with moving the robot by sending joint velocity commands. Run the test forward velocity controller launch file using the command line:
```bash
 ros2 launch ur_robot_driver test_forward_velocity_controller.launch.py
 ```

5. Additionally, you can visualize the communication graph using `rqt_graph`. Open a new terminal and enter the following command:

    ```bash
    rqt_graph
    ```

   This will display a graphical representation of the ROS 2 communication graph, allowing you to see the connections between nodes.

Now, you have successfully tested the communication between ROS 2 and the Universal Robots robot, and you can monitor joint positions and visualize the communication graph.

## Setting up ROS 2 Workspace and Cloning GitHub Repository

To use the provided ROS 2 packages, follow these steps to set up a workspace, clone the GitHub repository, and build the packages:

1. **Create a ROS 2 Workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

2. **Clone the GitHub Repository:**
    ```bash
    
    git clone https://github.com/ER-ROUGUI/Ros2_Ur_robot_vision.git src
    ```

3. **Build the Packages:**
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```

4. **Source the Setup Script:**
    ```bash
    source install/setup.bash
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    ```
    ```bash
    source ~/.bashrc
    ```
    

Now, your ROS 2 workspace is set up, the GitHub repository is cloned, and the packages are built. You are ready to use the installed packages in your ROS 2 environment.

## Explanation of my_controller Functionality
The my_controller package is designed to facilitate control and interaction with a Universal Robot UR3e using ROS 2 and Image-Based Visual Servoing (IBVS). The key components of this package include geometric and kinematic modeling, integration of the UR Robot Driver, and the implementation of visual servoing using either circle detection or QR code corner detection.


1. **Circle Detection vs. QR Code Corner Detection:**
   In the  `image_detection` and `image_detection_qr` nodes within the package, two different approaches are employed for visual  
   feature extraction:

   Circle Detection:

**The image_detection node:**  utilizes circle detection to identify the centers of circular features in the camera image. This is done using OpenCV's HoughCircles function.
While circle detection provides feature points, it may have limitations during visual servoing, leading to potential challenges.
QR Code Corner Detection:

**The image_detection_qr node:**  adopts a more robust approach by leveraging QR code corner detection.
QR code corner detection extracts the coordinates of the four corners of detected QR codes in the image. This method proves to be more reliable for visual servoing tasks.

**Computation of Error**
The primary objective during visual servoing is to minimize the error between the detected visual features and a set of fixed points. This error is computed using the `calculate_error` function, which compares the coordinates of the detected features (either circle centers or QR code corners) with the predefined fixed points.

**Normalized Coordinates**
To ensure consistent and accurate error calculations, the `normalized_coordinates` function is utilized. This function transforms pixel coordinates into normalized coordinates in meters. This conversion is crucial for precise error computations, allowing the comparison of visual feature positions with fixed points in a standardized metric.


## Control law

![Circle Detection](Ros2_Ur_robot_vision/my_controller/control_law.PNG.png)
