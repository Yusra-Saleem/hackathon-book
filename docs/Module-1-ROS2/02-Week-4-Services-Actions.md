---
sidebar_position: 2
sidebar_label: 'Week 4: ROS2 Services & Actions'
---

# Week 4: ROS2 Services & Actions - Advanced Communication Patterns

Building on last week's introduction to Nodes and Topics, this chapter explores two other crucial communication patterns in ROS2: **Services** and **Actions**. These patterns enable more complex and robust interactions between nodes, essential for building sophisticated robotic behaviors.

## Learning Objectives

By the end of this week, you should be able to:

1.  Understand the **request-response communication model** of ROS2 Services.
2.  Implement a basic **ROS2 Service server and client** in Python.
3.  Grasp the **long-running, goal-oriented communication model** of ROS2 Actions.
4.  Implement a basic **ROS2 Action server and client** in Python.
5.  Differentiate between Topics, Services, and Actions and choose the appropriate communication pattern for a given task.

## 1. ROS2 Services: Synchronous Request-Response

**ROS2 Services** provide a synchronous, request-response communication mechanism. Unlike topics, which are asynchronous and one-way, services are used when a node needs to send a request to another node and wait for a definite response.

Think of it like a function call across your robot's network. A client node sends a request, and a server node processes it and returns a result. This is ideal for tasks like:
*   Triggering a specific action (e.g., "take a picture", "move to point X").
*   Querying information (e.g., "what's the current battery level?").
*   Performing a computation that returns a single result.

### Defining a Service

Services are defined using `.srv` files within a ROS2 package. Let's create a simple service that adds two integers.

1.  **Create `my_robot_pkg/srv/AddTwoInts.srv`**:
    ```
    int64 a
    int64 b
    ---
    int64 sum
    ```
    The `---` separates the request fields from the response fields.

2.  **Update `package.xml`**: Add dependencies for `rosidl_default_generators` and `rosidl_default_runtime`.
    ```xml
    <!-- package.xml snippet -->
    <buildtool_depend>ament_cmake</buildtool_depend>
    <buildtool_depend>ament_python</buildtool_depend>
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    <!-- ... -->
    ```

3.  **Update `CMakeLists.txt`**: Add instructions to build the service interface.
    ```cmake
    # CMakeLists.txt snippet
    find_package(ament_cmake REQUIRED)
    find_package(rclpy REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "srv/AddTwoInts.srv"
    )

    # ...
    ament_python_install_package(${PROJECT_NAME})
    ```

### Implementing a Service Server (Python)

```python
# my_robot_pkg/my_robot_pkg/add_two_ints_server.py
import rclpy
from rclpy.node import Node
from my_robot_pkg.srv import AddTwoInts # Import our custom service type

class AddTwoIntsServer(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('AddTwoInts service server started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    server = AddTwoIntsServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Implementing a Service Client (Python)

```python
# my_robot_pkg/my_robot_pkg/add_two_ints_client.py
import rclpy
from rclpy.node import Node
from my_robot_pkg.srv import AddTwoInts

class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        self.get_logger().info(f'Sending request: a={a}, b={b}')

def main(args=None):
    rclpy.init(args=args)
    client = AddTwoIntsClient()
    # Example usage
    client.send_request(5, 7)

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().error(f'Service call failed: {e}')
            else:
                client.get_logger().info(f'Result of add_two_ints: for {client.req.a} + {client.req.b} = {response.sum}')
            break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Remember to update `setup.py` with entry points for both server and client, and then build and source your workspace before running.

## 2. ROS2 Actions: Goal-Oriented, Long-Running Tasks

**ROS2 Actions** are designed for long-running tasks that involve a goal, continuous feedback during execution, and a final result. They are built on top of topics and services, providing a more structured way to manage complex, goal-oriented behaviors.

Actions are ideal for tasks like:
*   **Navigation**: "Go to location X" (goal) with periodic updates on progress (feedback) and a final success/failure (result).
*   **Arm Manipulation**: "Pick up object Y" (goal) with updates on arm joint positions (feedback) and whether the object was successfully picked (result).
*   **Complex Sequences**: Tasks that might be preempted or require monitoring throughout their execution.

### Defining an Action

Actions are defined using `.action` files. Let's create an action for a "fibonacci" sequence generator.

1.  **Create `my_robot_pkg/action/Fibonacci.action`**:
    ```
    int32 order
    ---
    int32[] sequence
    ---
    int32[] partial_sequence
    ```
    The first `---` separates the goal from the result. The second `---` separates the result from the feedback.

2.  **Update `package.xml` and `CMakeLists.txt`**: Similar to services, you need to add dependencies and build instructions for action interfaces.

### Implementing an Action Server (Python)

```python
# my_robot_pkg/my_robot_pkg/fibonacci_action_server.py
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from my_robot_pkg.action import Fibonacci # Import our custom action type

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info('Fibonacci Action Server started.')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: {goal_handle.request.order}')

        sequence = [0, 1]
        # Append to the sequence until the requested order is achieved
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            sequence.append(sequence[i] + sequence[i-1])
            # Publish feedback
            feedback_msg = Fibonacci.Feedback()
            feedback_msg.partial_sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            time.sleep(0.5)

        goal_handle.succeed()

        # Populate result message
        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info(f'Result: {result.sequence}')
        return result

def main(args=None):
    rclpy.init(args=args)
    server = FibonacciActionServer()
    rclpy.spin(server)

if __name__ == '__main__':
    main()
```

### Implementing an Action Client (Python)

```python
# my_robot_pkg/my_robot_pkg/fibonacci_action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from my_robot_pkg.action import Fibonacci

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Sending goal: {order}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4: # GoalStatus.SUCCEEDED
            self.get_logger().info(f'Goal succeeded! Result: {result.sequence}')
        else:
            self.get_logger().info(f'Goal failed with status: {status}')

        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.partial_sequence}')

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10) # Request Fibonacci sequence up to order 10
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## 3. Choosing the Right Communication Pattern

| Pattern | Communication Model | Use Case | Characteristics |
| :------ | :------------------ | :------- | :-------------- |
| **Topics** | Asynchronous Publish-Subscribe | Continuous data streams (sensor readings, robot odometry) | One-way, many-to-many, non-blocking |
| **Services** | Synchronous Request-Response | Single-shot commands, querying data, short computations | Two-way, one-to-one, blocking (client waits for response) |
| **Actions** | Goal-Oriented, Long-Running | Navigation, complex manipulation, tasks with feedback | Two-way, goal-feedback-result, cancellable, provides progress updates |

Selecting the appropriate communication pattern is critical for designing efficient and robust ROS2 applications. Consider the nature of the interaction: is it a continuous stream, a one-off request, or a long-running task with intermediate feedback?

## Conclusion

This week, you've gained a deeper understanding of ROS2's advanced communication patterns: Services and Actions. You can now implement synchronous request-response interactions and manage complex, long-running tasks with continuous feedback. With Topics, Services, and Actions in your toolkit, you are well-equipped to design sophisticated and distributed robotic software architectures. Next, we'll explore how to describe and interact with the robot's physical structure using URDF and the `rclpy` library.

## Further Reading

*   **ROS2 Documentation**: [Understanding ROS2 Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
*   **ROS2 Documentation**: [Understanding ROS2 Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
