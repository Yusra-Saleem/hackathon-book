---
sidebar_position: 3
sidebar_label: 'Week 5: URDF & rclpy for Robot Description'
---

# Week 5: URDF & `rclpy` for Robot Description and Control

This week, we dive into how to formally describe a robot's physical structure and how to control it using ROS2's Python client library, `rclpy`. Understanding these concepts is fundamental for simulating robots, performing kinematic calculations, and ultimately bringing your physical AI creations to life.

## Learning Objectives

By the end of this week, you should be able to:

1.  Understand the purpose and structure of **URDF (Unified Robot Description Format)**.
2.  Define **links and joints** in a URDF file.
3.  Visualize a robot model using URDF in **RViz2**.
4.  Develop a basic **`rclpy` node to publish joint states**.
5.  Integrate `robot_state_publisher` for kinematic transformations.

## 1. Unified Robot Description Format (URDF)

**URDF** is an XML-based file format used in ROS2 to describe the physical properties of a robot. It allows you to define the robot's kinematics (how its parts are connected and can move), visuals (how it looks), and collision properties (its physical shape for collision detection).

### Key URDF Elements:

*   **`<robot>`**: The root element, defining the robot's name.
*   **`<link>`**: Represents a rigid body part of the robot (e.g., a chassis, a wheel, an arm segment).
    *   Can contain `visual`, `collision`, and `inertial` sub-elements.
    *   `visual`: Defines the 3D model (e.g., `<mesh>`, `<box>`, `<cylinder>`) and material properties.
    *   `collision`: Defines the geometry used for physics-based collision detection.
    *   `inertial`: Defines mass and inertia tensor, critical for realistic simulation.
*   **`<joint>`**: Represents the kinematic connection between two links.
    *   Connects a `parent` link to a `child` link.
    *   `type`: Specifies the joint's motion (e.g., `revolute` for rotating, `prismatic` for sliding, `fixed` for rigid connection).
    *   `origin`: Defines the 3D position and orientation of the child link relative to the parent.
    *   `axis`: For revolute/prismatic joints, defines the axis of rotation/translation.
    *   `limit`: Sets the range of motion and effort/velocity limits.

### Simple URDF Example (Two-Link Arm)

Let's create a basic URDF for a two-link robotic arm. This file would typically be saved as `my_robot.urdf` in a `urdf` subdirectory of your ROS2 package.

```xml
<?xml version="1.0"?>
<robot name="two_link_arm">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="link2">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.15"/>
      </geometry>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
  </link>

</robot>
```

### Visualizing URDF in RViz2

To see your robot in action, you can use RViz2, the ROS2 visualization tool. You'll typically need `robot_state_publisher` to read your URDF and publish the robot's kinematic state as `/tf` (transform) messages.

1.  **Install necessary packages** (if not already present):
    ```bash
    sudo apt install ros-<ROS_DISTRO>-joint-state-publisher-gui ros-<ROS_DISTRO>-robot-state-publisher ros-<ROS_DISTRO>-rviz2
    ```
2.  **Create a launch file** (e.g., `display_robot.launch.py` in your package's `launch` directory) to load the URDF and start `robot_state_publisher` and `rviz2`.
    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        # Get URDF path
        urdf_path = os.path.join(
            get_package_share_directory('my_robot_pkg'),
            'urdf',
            'two_link_arm.urdf')

        with open(urdf_path, 'r') as infp:
            robot_desc = infp.read()

        return LaunchDescription([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_desc}]
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(
                    get_package_share_directory('my_robot_pkg'),
                    'rviz',
                    'urdf_config.rviz')]
            )
        ])
    ```
    *   You'll also need an RViz configuration file (e.g., `urdf_config.rviz`) in `my_robot_pkg/rviz/`.

3.  **Run the launch file**:
    ```bash
    ros2 launch my_robot_pkg display_robot.launch.py
    ```

## 2. Controlling Joints with `rclpy`

To animate or control your robot, you'll publish `JointState` messages to the `/joint_states` topic. The `robot_state_publisher` node then uses these joint states, along with the URDF, to calculate the full kinematic state of the robot and publish it as TF (Transform) messages, which RViz2 subscribes to for visualization.

### `rclpy` Joint State Publisher Node

```python
# my_robot_pkg/my_robot_pkg/joint_state_publisher_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        self.get_logger().info('Joint State Publisher Node started!')

        self.joint1_angle = 0.0
        self.joint2_angle = 0.0

    def timer_callback(self):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint1', 'joint2'] # Must match URDF joint names

        # Oscillate joint angles for demonstration
        self.joint1_angle = 0.5 * math.sin(self.get_clock().now().nanoseconds / 1e9)
        self.joint2_angle = 0.5 * math.cos(self.get_clock().now().nanoseconds / 1e9)

        joint_state_msg.position = [self.joint1_angle, self.joint2_angle]
        joint_state_msg.velocity = [] # Optional
        joint_state_msg.effort = []   # Optional

        self.publisher_.publish(joint_state_msg)
        # self.get_logger().info(f'Publishing: joint1={self.joint1_angle:.2f}, joint2={self.joint2_angle:.2f}')

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Joint State Publisher

1.  Update `my_robot_pkg/setup.py` to add an entry point for `joint_state_publisher_node`.
2.  Build your workspace (`colcon build --packages-select my_robot_pkg`).
3.  Source your workspace (`source install/setup.bash`).
4.  In one terminal, run your `display_robot.launch.py` (which starts `robot_state_publisher` and `rviz2`).
5.  In another terminal, run your joint state publisher node:
    ```bash
    ros2 run my_robot_pkg joint_state_publisher_node
    ```

In RViz2, you should now see your two-link arm moving as the joint states are published.

## Conclusion

This week, you've mastered the fundamentals of describing your robot's physical form using URDF and dynamically controlling its joints through `rclpy` and the `joint_states` topic. This ability to represent and manipulate robot kinematics is essential for any physical AI application, from simulation to real-world deployment. Next, we will transition into Module 2, focusing on creating digital twins and leveraging simulation environments.

## Further Reading

*   **ROS2 Documentation**: [URDF Overview](https://docs.ros.org/en/humble/Tutorials/URDF/URDF-Overview.html)
*   **ROS2 Documentation**: [Using URDF with robot_state_publisher](https://docs.ros.org/en/humble/Tutorials/Advanced/URDF/Using-URDF-with-RobotStatePublisher.html)
*   **ROS2 Tutorials**: [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Publisher-And-Subscriber-Python.html)
