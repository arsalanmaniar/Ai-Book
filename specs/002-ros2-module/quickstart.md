# Quickstart Guide: Module 1 "The Robotic Nervous System (ROS 2)"

## Prerequisites

Before starting this module, ensure you have:

- Ubuntu 22.04 LTS installed (or a compatible Linux distribution)
- ROS 2 Humble Hawksbill installed
- Python 3.8 or higher
- Basic knowledge of Python programming
- Familiarity with Linux command line

### Installing ROS 2 Humble Hawksbill

1. Set up your sources.list:
```bash
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

2. Install ROS 2 packages:
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

3. Install Python development tools:
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
```

4. Source the ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
```

## Setting Up Your Workspace

1. Create a new workspace directory:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

2. Source ROS 2 environment (add to ~/.bashrc to make permanent):
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Running Your First Example

1. Create a simple publisher node (`publisher_example.py`):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. Run the publisher in one terminal:
```bash
cd ~/ros2_ws
source install/setup.bash
python3 publisher_example.py
```

3. In another terminal, run the subscriber:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 topic echo /topic std_msgs/msg/String
```

You should see the published messages appearing in the subscriber terminal.

## Exploring URDF

1. Create a simple URDF file (`simple_robot.urdf`):
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <link name="sensor_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="sensor_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_link"/>
    <origin xyz="0.2 0 0"/>
  </joint>
</robot>
```

2. View the URDF in RViz:
```bash
# In one terminal, run the robot state publisher
cd ~/ros2_ws
source install/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat simple_robot.urdf)

# In another terminal, run RViz
ros2 run rviz2 rviz2
```

In RViz, add a RobotModel display and set the topic to `/robot_description` to visualize your robot.

## Next Steps

1. Complete the exercises in each chapter:
   - Chapter 1: ROS 2 Middleware Concepts
   - Chapter 2: Nodes, Topics, and Services
   - Chapter 3: Python Agent Integration and URDF Fundamentals

2. Practice creating your own ROS 2 nodes with different communication patterns

3. Experiment with more complex URDF models for humanoid robots

4. Explore Quality of Service (QoS) settings to understand their impact on communication reliability

## Troubleshooting

- If you get "command not found" errors, make sure you've sourced the ROS 2 environment
- If nodes can't communicate, check that they're on the same ROS domain (use `ROS_DOMAIN_ID` to set domain)
- If URDF doesn't display properly, validate the XML syntax and ensure all links are properly connected