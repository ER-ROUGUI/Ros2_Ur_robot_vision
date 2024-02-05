# Robotic Control with ROS2 and Visual Servoing

This project involves controlling a Universal Robot UR3e using ROS2 with the Humble distribution. It includes creating geometric and kinematic models of the robot 
based on its DH parameters, implementing Image-Based Visual Servoing (IBVS), and explaining the installation and configuration steps
## Installation Steps
### Installation ROS2
To begin, you'll need to install Ubuntu (version 22.04). Next, follow the steps outlined in the [official ROS2 Humble installation tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

Run the following command to add the line to your `~/.bashrc` file:

   ```
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
'''

### Installtion the driver


## Installing Colcon for Building Packages

To build ROS 2 packages, we need to install Colcon. Follow these steps:

Run the following command to install Colcon:

    ```
    sudo apt install python3-colcon-common-extensions
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



 To install the ** ur_robot_driver ** you can choose to either use the binary packages or install from source. To install using binary packages, use the following command:

    ```bash
    sudo apt-get install ros-humble-ur
    ```

This will install the UR Robot Driver for ROS 2 Humble using binary packages. Adjust the installation method based on your preferences and requirements.
