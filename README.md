# Robotic Control with ROS2 and Visual Servoing

This project involves controlling a Universal Robot UR3e using ROS2 with the Humble distribution. It includes creating geometric and kinematic models of the robot 
based on its DH parameters, implementing Image-Based Visual Servoing (IBVS), and explaining the installation and configuration steps
## Installation Steps
### Installation ROS2
To begin, you'll need to install Ubuntu (version 22.04). Next, follow the steps outlined in the [official ROS2 Humble installation tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

Run the following command to add the line to your `~/.bashrc` file:




```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```


### Installtion the driver


## Installing Colcon for Building Packages

To build ROS 2 packages, we need to install Colcon. Follow these steps:

Run the following command to install Colcon:

```
sudo apt install python3-colcon-common-extensions python3-vcstool
```

Enable Colcon autocompletion by adding the following line to your `~/.bashrc` file:

```
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```

Apply the changes by running:

```
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

    Ensure to replace `<ur_type>` with the specific type of your Universal Robots robot and set the correct robot IP address (e.g., `192.168.1.101`). This command launches the UR robot driver and initializes communication with ROS 2.

3. Optionally, visualize the state of the robot in Rviz.

Now, your UR robot should be set up and ready to communicate with


