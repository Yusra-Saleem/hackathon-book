---
sidebar_position: 1
sidebar_label: 'Gazebo Simulation Setup'
---

# Gazebo Simulation Setup

This chapter focuses on setting up and interacting with Gazebo, a powerful 3D robotics simulator. Gazebo allows you to accurately simulate complex robot systems in realistic environments, providing a safe and cost-effective platform for developing and testing robotic algorithms before deploying them on physical hardware.

## Introduction to Gazebo

Gazebo is an open-source multi-robot simulator that accurately and efficiently simulates populations of robots in complex indoor and outdoor environments. It offers:
-   **Physics Engine**: High-fidelity physics engine (ODE, Bullet, DART, Simbody) for realistic interactions.
-   **3D Graphics**: Rendered environments and robot models for visual feedback.
-   **Sensors Simulation**: Realistic simulation of various sensors like cameras, LiDAR, depth sensors, IMUs, etc.
-   **Plugin Architecture**: Extensible design allowing users to create custom plugins for robot control, sensor models, and world interactions.
-   **ROS2 Integration**: Seamless integration with ROS2 through various packages and plugins, allowing direct control of simulated robots using ROS2 nodes.

## Installing Gazebo

Gazebo typically comes in versions tied to ROS2 distributions (e.g., Foxy, Galactic, Humble). We'll assume a ROS2 Humble installation.

```bash
# Update your package list
sudo apt update

# Install Gazebo Garden (default for Humble)
sudo apt install ros-humble-gazebo-ros-pkgs

# This will install Gazebo, gazebo_ros_pkgs, and their dependencies.
# You might need to install Gazebo separately if using a different ROS2 distribution or a standalone Gazebo.
# For standalone Gazebo:
# sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
# wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
# sudo apt update
# sudo apt install gazebo
```

## Creating a Simple Gazebo World

Gazebo worlds are defined using SDF (Simulation Description Format) files. SDF is an XML format used to describe objects and environments for robot simulators.

1.  **Create a `worlds` directory in your ROS2 package**:
    ```bash
    mkdir -p my_robot_pkg/worlds
    ```

2.  **Create `empty_world.sdf` inside `my_robot_pkg/worlds/`**:
    ```xml
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="default">
        <light name="sun" type="directional">
          <cast_shadows>1</cast_shadows>
          <pose>0 0 10 0 -0 0</pose>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
          <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
          </attenuation>
          <direction>-0.5 -0.5 -1</direction>
        </light>

        <include>
          <uri>model://ground_plane</uri>
        </include>
        <include>
          <uri>model://sun</uri>
        </include>
      </world>
    </sdf>
    ```
    This defines an empty world with a ground plane and a sun.

## Launching Gazebo with ROS2

You can launch Gazebo from a ROS2 launch file, which makes it easy to integrate with your robot's nodes.

1.  **Update `CMakeLists.txt` and `package.xml`**: Ensure your package is set up for `ros_environment` and `gazebo_ros` if you plan to use Gazebo plugins later. For now, we'll focus on launching.

2.  **Create a `launch` directory in your package**:
    ```bash
    mkdir -p my_robot_pkg/launch
    ```

3.  **Create `start_gazebo.launch.py` inside `my_robot_pkg/launch/`**:
    ```python
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node
    import os
    from ament_index_python.packages import get_package_share_directory

    def generate_launch_description():
        gazebo_ros_dir = get_package_share_directory('gazebo_ros')
        my_robot_pkg_dir = get_package_share_directory('my_robot_pkg')

        # Path to your custom world file
        world_file_name = 'empty_world.sdf'
        world_path = os.path.join(my_robot_pkg_dir, 'worlds', world_file_name)

        # Launch Gazebo
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items(),
        )

        return LaunchDescription([
            gazebo_launch
        ])
    ```

4.  **Update `setup.py`**: Add the launch file to `data_files`.
    ```python
    # ... (previous setup.py content)
        data_files=[
            # ... (other data_files)
            ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
            ('share/' + package_name + '/worlds', glob(os.path.join('worlds', '*.sdf'))),
        ],
        # ...
    )
    ```

5.  **Build and Run**:
    ```bash
    cd <your_ros2_workspace>
    colcon build --packages-select my_robot_pkg
    source install/setup.bash

    ros2 launch my_robot_pkg start_gazebo.launch.py
    ```
    This should launch Gazebo with your empty world.

## Spawning a Robot in Gazebo

To see your robot in Gazebo, you'll need to spawn it using the `spawn_entity.py` script from `gazebo_ros`. This script takes your robot's URDF/SDF description and places it in the simulation.

1.  **Place your robot's URDF/SDF file**: For instance, place your `two_link_arm.urdf` (from Week 5) in `my_robot_pkg/urdf/`.

2.  **Modify `start_gazebo.launch.py` to include robot spawning**:
    ```python
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node
    import os
    from ament_index_python.packages import get_package_share_directory

    def generate_launch_description():
        gazebo_ros_dir = get_package_share_directory('gazebo_ros')
        my_robot_pkg_dir = get_package_share_directory('my_robot_pkg')

        # Path to your custom world file
        world_file_name = 'empty_world.sdf'
        world_path = os.path.join(my_robot_pkg_dir, 'worlds', world_file_name)

        # Path to your robot URDF file
        urdf_file_name = 'two_link_arm.urdf' # Assuming your URDF is here
        urdf_path = os.path.join(my_robot_pkg_dir, 'urdf', urdf_file_name)

        with open(urdf_path, 'r') as infp:
            robot_desc = infp.read()

        # Launch Gazebo
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items(),
        )

        # Node to publish the robot description
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        )

        # Node to spawn the robot in Gazebo
        spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'two_link_arm', # Must match your robot name in URDF
                '-x', '0.0', '-y', '0.0', '-z', '0.1' # Initial position
            ],
            output='screen'
        )

        return LaunchDescription([
            gazebo_launch,
            robot_state_publisher_node,
            spawn_entity_node
        ])
    ```
    Remember to add your `urdf` directory and the URDF file to `data_files` in `setup.py`.

3.  **Build and Run**:
    ```bash
    cd <your_ros2_workspace>
    colcon build --packages-select my_robot_pkg
    source install/setup.bash

    ros2 launch my_robot_pkg start_gazebo.launch.py
    ```
    Gazebo should launch, and you should see your `two_link_arm` robot spawned in the empty world.

## Conclusion

Gazebo is an indispensable tool for robotics development, providing a high-fidelity simulation environment. This chapter covered the basics of installing Gazebo, creating simple worlds, and spawning your URDF-defined robots within the simulation, all integrated with ROS2 launch files. In subsequent chapters, we'll explore more advanced Gazebo features, including sensor simulation and custom plugins.
