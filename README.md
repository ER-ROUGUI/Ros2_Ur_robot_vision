# Robotic Control with ROS2 and Visual Servoing

This project involves controlling a Universal Robot UR3e using ROS2 with the Humble distribution. It includes creating geometric and kinematic models of the robot 
based on its DH parameters, implementing Image-Based Visual Servoing (IBVS), and explaining the installation and configuration steps
## Installation Steps

To begin, you'll need to install Ubuntu (version 22.04). Next, follow the steps outlined in the [official ROS2 Humble installation tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

Run the following command to add the line to your `~/.bashrc` file:

   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
