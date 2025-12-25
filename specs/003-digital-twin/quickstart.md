# Quickstart Guide: Module 2 "The Digital Twin (Gazebo & Unity)"

## Prerequisites

Before starting this module, ensure you have:

- Ubuntu 22.04 LTS installed (for Gazebo) or Windows/Mac (for Unity)
- Gazebo Harmonic installed (for physics simulation)
- Unity 2022.3 LTS installed (for rendering and interaction)
- ROS 2 Humble Hawksbill installed
- Basic knowledge of ROS 2 concepts (from Module 1)

### Installing Gazebo Harmonic

1. Set up your sources.list (if not already done for Module 1):
```bash
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

2. Install Gazebo Harmonic:
```bash
sudo apt update
sudo apt install gazebo libgazebo-dev
```

3. Install Gazebo ROS packages:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

### Installing Unity 2022.3 LTS

1. Download Unity Hub from the Unity website
2. Install Unity Hub and use it to install Unity 2022.3 LTS
3. During installation, select the Universal Render Pipeline (URP) or High Definition Render Pipeline (HDRP) package for advanced rendering

## Setting Up Your Simulation Workspace

1. Create a new simulation workspace directory:
```bash
mkdir -p ~/simulation_ws/src
cd ~/simulation_ws
```

2. Source ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Running Your First Gazebo Simulation

1. Create a simple robot model file (`simple_humanoid.urdf`):
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="head_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.4"/>
  </joint>
</robot>
```

2. Create a simple world file (`simple_room.world`):
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_room">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Room walls -->
    <model name="wall_1">
      <pose>3 0 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 6 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 6 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="table">
      <pose>-1 0 0.4 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 0.8 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.2 0.1 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

3. Launch Gazebo with your world:
```bash
cd ~/simulation_ws
gazebo simple_room.world
```

4. In another terminal, spawn your robot:
```bash
# First, convert URDF to SDF format or use ROS 2 tools to spawn
# For this example, we'll load it directly into Gazebo
```

## Creating Your First Unity Scene

1. Open Unity Hub and create a new 3D project
2. Create a new scene and save it as "BasicHumanoidScene"
3. Add a 3D cube to represent the robot base and a sphere for the head
4. Add a directional light to illuminate the scene
5. Position the objects to match your robot configuration

## Adding Sensor Simulation to Gazebo

1. Create a LiDAR sensor configuration in your URDF:
```xml
<gazebo reference="head">
  <sensor name="lidar_sensor" type="ray">
    <pose>0.05 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot1</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

2. To visualize the LiDAR data in ROS 2:
```bash
# Terminal 1: Start your simulation
ros2 launch your_simulation_package your_launch_file.launch.py

# Terminal 2: Visualize LiDAR data
sudo apt install ros-humble-rviz2
ros2 run rviz2 rviz2
# In RViz, add a LaserScan display and set the topic to /robot1/scan
```

## Next Steps

1. Complete the exercises in each chapter:
   - Chapter 4: Gazebo Physics Simulation
   - Chapter 5: Unity Rendering and Human-Robot Interaction
   - Chapter 6: Sensor Simulation

2. Practice creating more complex humanoid models with proper joint configurations

3. Experiment with different physics parameters to see their effects

4. Explore Unity's rendering capabilities for more realistic robot visualization

5. Integrate sensor data from Gazebo into Unity for a complete digital twin experience

## Troubleshooting

- If Gazebo crashes or behaves unexpectedly, check that your URDF/SDF files are properly formatted
- If Unity scenes appear dark, adjust lighting and material settings
- If sensor data is not publishing correctly, verify ROS 2 package installations
- For physics issues, ensure proper mass and inertia values in your robot models