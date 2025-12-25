---
title: Sensor Simulation Exercises
sidebar_label: Exercises
---

# Sensor Simulation Exercises

**Module**: Module 2 "The Digital Twin (Gazebo & Unity)"
**Chapter**: 6 - Sensor Simulation
**Section**: Exercises
**Created**: 2025-12-20
**Last Updated**: 2025-12-20

## Overview

This section provides hands-on exercises to reinforce your understanding of sensor simulation concepts. The exercises range from basic configuration tasks to complex multi-sensor fusion implementations. Each exercise builds on the theoretical concepts covered in previous sections and provides practical experience with sensor simulation in Gazebo and ROS.

## Exercise 1: LiDAR Sensor Configuration

### Objective
Configure a 2D LiDAR sensor in a Gazebo environment and validate its performance characteristics.

### Prerequisites
- Basic understanding of URDF and Gazebo
- ROS installation with Gazebo plugins
- Basic knowledge of LiDAR principles

### Tasks
1. Create a simple robot model with a LiDAR sensor attached
2. Configure the LiDAR with the following specifications:
   - Range: 0.1m to 30m
   - Angular resolution: 1 degree
   - Field of view: 270 degrees
   - Update rate: 10 Hz
3. Add appropriate noise models to the sensor
4. Test the sensor in a simple environment with various obstacles
5. Validate the sensor output using RViz

### Implementation Steps

1. Create a URDF file for a simple robot with LiDAR:
```xml
<?xml version="1.0"?>
<robot name="lidar_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- LiDAR link -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base and LiDAR -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugin for LiDAR -->
  <gazebo reference="lidar_link">
    <sensor type="ray" name="lidar_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>270</samples>
            <resolution>1</resolution>
            <min_angle>-2.356</min_angle> <!-- -135 degrees -->
            <max_angle>2.356</max_angle>   <!-- 135 degrees -->
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_plugin" filename="libgazebo_ros_laser.so">
        <topicName>scan</topicName>
        <frameName>lidar_link</frameName>
        <min_range>0.1</min_range>
        <max_range>30.0</max_range>
        <gaussianNoise>0.01</gaussianNoise>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

2. Create a simple world file for testing:
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="lidar_test_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Test obstacles -->
    <model name="wall_1">
      <pose>2 0 1 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>4 0.1 2</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>4 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="box_obstacle">
      <pose>-1 1 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

3. Launch the simulation and verify the LiDAR data:
```bash
roslaunch gazebo_ros empty_world.launch world_name:=path/to/your/world
# In another terminal:
rostopic echo /scan
# Visualize in RViz
rosrun rviz rviz
```

### Validation Criteria
- LiDAR scan data shows correct range and angular resolution
- Obstacles are detected at expected distances
- Noise characteristics are visible in the data
- Frame rate matches configured update rate

## Exercise 2: Depth Camera Integration

### Objective
Integrate a depth camera into a robot model and process RGB-D data for obstacle detection.

### Tasks
1. Add a depth camera to your robot model
2. Configure the camera with appropriate parameters
3. Process depth images to detect obstacles
4. Visualize the results in RViz

### Implementation Steps

1. Modify your URDF to include a depth camera:
```xml
<!-- Depth camera link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.04 0.15 0.04"/>
    </geometry>
    <material name="black">
      <color rgba="0.1 0.1 0.1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.04 0.15 0.04"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor type="depth" name="camera_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <cameraName>camera</cameraName>
      <frameName>camera_link</frameName>
      <baseline>0.1</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      <pointCloudCutoff>0.1</pointCloudCutoff>
      <pointCloudCutoffMax>5.0</pointCloudCutoffMax>
      <CxPrime>0</CxPrime>
      <Cx>320.5</Cx>
      <Cy>240.5</Cy>
      <focalLength>525.0</focalLength>
      <depthImageTopicName>/camera/depth/image_raw</depthImageTopicName>
      <depthImageCameraInfoTopicName>/camera/depth/camera_info</depthImageCameraInfoTopicName>
      <rgbImageTopicName>/camera/rgb/image_raw</rgbImageTopicName>
      <rgbCameraInfoTopicName>/camera/rgb/camera_info</rgbCameraInfoTopicName>
      <pointCloudTopicName>/camera/depth/points</pointCloudTopicName>
    </plugin>
  </sensor>
</gazebo>
```

2. Create a Python script to process depth images:
```python
#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DepthProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.obstacle_pub = rospy.Publisher("/obstacles", Image, queue_size=1)

    def depth_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

            # Process depth image to detect obstacles
            # Convert to 8-bit for visualization
            depth_normalized = cv2.normalize(cv_depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)

            # Create obstacle mask (obstacles within 2 meters)
            obstacle_mask = (cv_depth < 2.0) & (cv_depth > 0.1)
            obstacle_img = np.zeros_like(depth_normalized)
            obstacle_img[obstacle_mask] = 255

            # Convert back to ROS image
            obstacle_msg = self.bridge.cv2_to_imgmsg(obstacle_img, encoding='mono8')
            obstacle_msg.header = msg.header
            self.obstacle_pub.publish(obstacle_msg)

        except Exception as e:
            rospy.logerr("Error processing depth image: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('depth_processor')
    processor = DepthProcessor()
    rospy.spin()
```

### Validation Criteria
- Depth camera produces RGB and depth images
- Obstacle detection algorithm correctly identifies close objects
- Point cloud data is generated and accessible
- Visualization shows obstacle locations

## Exercise 3: IMU Integration and State Estimation

### Objective
Integrate an IMU sensor and implement a simple state estimation system.

### Tasks
1. Add an IMU to your robot model
2. Implement a complementary filter for attitude estimation
3. Validate the estimated attitude against ground truth

### Implementation Steps

1. Add IMU to your robot URDF:
```xml
<!-- IMU link -->
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.01 0.01 0.01"/>
    </geometry>
    <material name="green">
      <color rgba="0 1 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.01 0.01 0.01"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor type="imu" name="imu_sensor">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>false</visualize>
    <topic>__default_topic__</topic>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <bodyName>imu_link</bodyName>
      <topicName>imu/data</topicName>
      <serviceName>imu/service</serviceName>
      <gaussianNoise>0.0017</gaussianNoise>
      <updateRate>100.0</updateRate>
      <frameName>imu_link</frameName>
      <initialOrientationAsReference>false</initialOrientationAsReference>
    </plugin>
  </sensor>
</gazebo>
```

2. Implement a complementary filter for attitude estimation:
```python
#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import quaternion_multiply, quaternion_conjugate, quaternion_from_euler
import math

class ComplementaryFilter:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = rospy.Time.now()

        # Filter parameters
        self.alpha = 0.98  # Complementary filter constant
        self.gyro_bias = np.array([0.0, 0.0, 0.0])

        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        self.attitude_pub = rospy.Publisher("/estimated_attitude", Vector3Stamped, queue_size=1)

    def imu_callback(self, msg):
        current_time = msg.header.stamp
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        if dt <= 0:
            return

        # Extract accelerometer data
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        # Extract gyroscope data (remove bias)
        gyro_x = msg.angular_velocity.x - self.gyro_bias[0]
        gyro_y = msg.angular_velocity.y - self.gyro_bias[1]
        gyro_z = msg.angular_velocity.z - self.gyro_bias[2]

        # Calculate attitude from accelerometer
        acc_roll = math.atan2(acc_y, acc_z)
        acc_pitch = math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2))

        # Integrate gyroscope data
        self.roll += gyro_x * dt
        self.pitch += gyro_y * dt
        self.yaw += gyro_z * dt

        # Apply complementary filter
        self.roll = self.alpha * (self.roll + gyro_x * dt) + (1 - self.alpha) * acc_roll
        self.pitch = self.alpha * (self.pitch + gyro_y * dt) + (1 - self.alpha) * acc_pitch

        # Publish estimated attitude
        attitude_msg = Vector3Stamped()
        attitude_msg.header.stamp = current_time
        attitude_msg.header.frame_id = "base_link"
        attitude_msg.vector.x = self.roll
        attitude_msg.vector.y = self.pitch
        attitude_msg.vector.z = self.yaw
        self.attitude_pub.publish(attitude_msg)

if __name__ == '__main__':
    rospy.init_node('complementary_filter')
    cf = ComplementaryFilter()
    rospy.spin()
```

### Validation Criteria
- IMU data is published at expected rate
- Complementary filter produces stable attitude estimates
- Estimated attitude matches simulated ground truth when available
- Filter responds appropriately to robot motion

## Exercise 4: Multi-Sensor Fusion

### Objective
Combine data from LiDAR, camera, and IMU sensors to create a more robust perception system.

### Tasks
1. Integrate all three sensors into a single robot model
2. Implement a simple Kalman filter for sensor fusion
3. Compare fused estimates with individual sensor estimates
4. Analyze the improvement in accuracy and robustness

### Implementation Steps

1. Combine all sensors in a single URDF:
```xml
<?xml version="1.0"?>
<robot name="multi_sensor_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.2"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.2" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- LiDAR -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Camera -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.04 0.15 0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>

  <!-- IMU -->
  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins for all sensors -->
  <gazebo reference="lidar_link">
    <sensor type="ray" name="lidar_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>270</samples>
            <resolution>1</resolution>
            <min_angle>-2.356</min_angle>
            <max_angle>2.356</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_plugin" filename="libgazebo_ros_laser.so">
        <topicName>scan</topicName>
        <frameName>lidar_link</frameName>
        <gaussianNoise>0.01</gaussianNoise>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="camera_link">
    <sensor type="depth" name="camera_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
        <cameraName>camera</cameraName>
        <frameName>camera_link</frameName>
        <pointCloudTopicName>/camera/depth/points</pointCloudTopicName>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor type="imu" name="imu_sensor">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <bodyName>imu_link</bodyName>
        <topicName>imu/data</topicName>
        <gaussianNoise>0.0017</gaussianNoise>
        <updateRate>100.0</updateRate>
        <frameName>imu_link</frameName>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

2. Implement a simple Kalman filter for position fusion:
```python
#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class SensorFusion:
    def __init__(self):
        # State vector: [x, y, vx, vy]
        self.x = np.zeros(4)

        # Covariance matrix
        self.P = np.eye(4) * 1000.0  # Start with high uncertainty

        # Process noise
        self.Q = np.eye(4) * 0.1

        # Measurement noise for different sensors
        self.R_lidar = np.eye(2) * 0.5  # LiDAR position measurement noise
        self.R_imu = np.eye(2) * 0.2    # IMU velocity measurement noise

        # Measurement matrices
        self.H_lidar = np.array([[1, 0, 0, 0],
                                [0, 1, 0, 0]])  # Position measurements
        self.H_imu = np.array([[0, 0, 1, 0],
                              [0, 0, 0, 1]])   # Velocity measurements

        # Subscribe to sensor data
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_callback)

        # Publisher for fused estimate
        self.pose_pub = rospy.Publisher("/fused_pose", PoseStamped, queue_size=1)

        # Timer for prediction step
        self.timer = rospy.Timer(rospy.Duration(0.01), self.prediction_step)  # 100 Hz
        self.last_time = rospy.Time.now()

    def prediction_step(self, event):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        if dt <= 0:
            return

        # State transition matrix (constant velocity model)
        F = np.array([[1, 0, dt, 0],
                     [0, 1, 0, dt],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

        # Predict state
        self.x = F.dot(self.x)

        # Predict covariance
        self.P = F.dot(self.P).dot(F.T) + self.Q

    def lidar_callback(self, msg):
        # Simple method to get robot position from LiDAR scan
        # This is a placeholder - in reality, you'd use scan matching or localization
        # For this exercise, we'll simulate a position measurement

        # In a real scenario, you would:
        # 1. Perform scan matching against a map
        # 2. Use landmark detection if known landmarks are in view
        # 3. Use other localization techniques

        # For simulation purposes, let's assume we get position measurements
        # This would come from a localization node in practice
        pass

    def imu_callback(self, msg):
        # Get orientation from IMU to transform local measurements to global frame
        orientation = msg.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = euler_from_quaternion(quaternion)

        # Extract linear acceleration
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y

        # Transform to global frame (simplified)
        global_acc_x = acc_x * np.cos(euler[2]) - acc_y * np.sin(euler[2])
        global_acc_y = acc_x * np.sin(euler[2]) + acc_y * np.cos(euler[2])

        # Integrate to get velocity
        dt = 0.01  # Assuming 100 Hz IMU
        vx = global_acc_x * dt
        vy = global_acc_y * dt

        # Create measurement vector
        z = np.array([vx, vy])

        # Innovation
        y = z - self.H_imu.dot(self.x)

        # Innovation covariance
        S = self.H_imu.dot(self.P).dot(self.H_imu.T) + self.R_imu

        # Kalman gain
        K = self.P.dot(self.H_imu.T).dot(np.linalg.inv(S))

        # Update state
        self.x = self.x + K.dot(y)

        # Update covariance
        I = np.eye(len(self.x))
        self.P = (I - K.dot(self.H_imu)).dot(self.P)

        # Publish fused estimate
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.x[0]
        pose_msg.pose.position.y = self.x[1]
        pose_msg.pose.orientation.w = 1.0  # Simplified
        self.pose_pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('sensor_fusion')
    sf = SensorFusion()
    rospy.spin()
```

### Validation Criteria
- All sensors are properly integrated in the simulation
- Kalman filter successfully combines sensor data
- Fused estimates show improved accuracy compared to individual sensors
- System handles sensor failures gracefully

## Exercise 5: Sensor Validation and Quality Assessment

### Objective
Implement validation techniques to assess the quality and reliability of sensor data.

### Tasks
1. Create validation algorithms for each sensor type
2. Implement sensor health monitoring
3. Design failure detection and recovery mechanisms

### Implementation Steps

1. Create a sensor validation node:
```python
#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Image, Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class SensorValidator:
    def __init__(self):
        # Initialize validation parameters
        self.lidar_stats = {'min_range': float('inf'), 'max_range': 0, 'mean_range': 0, 'count': 0}
        self.imu_stats = {'acc_norm': 0, 'gyro_norm': 0, 'count': 0}

        # Subscribe to sensor data
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.validate_lidar)
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.validate_imu)

        # Diagnostic publisher
        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

        # Timer for periodic validation
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)

    def validate_lidar(self, msg):
        # Validate LiDAR data
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]

        if len(valid_ranges) > 0:
            self.lidar_stats['min_range'] = min(self.lidar_stats['min_range'], min(valid_ranges))
            self.lidar_stats['max_range'] = max(self.lidar_stats['max_range'], max(valid_ranges))
            self.lidar_stats['mean_range'] = sum(valid_ranges) / len(valid_ranges)
            self.lidar_stats['count'] += 1

        # Check for sensor health
        if len(valid_ranges) < len(msg.ranges) * 0.9:  # More than 10% invalid readings
            rospy.logwarn("LiDAR sensor has high percentage of invalid readings")

    def validate_imu(self, msg):
        # Validate IMU data
        acc_norm = np.sqrt(msg.linear_acceleration.x**2 +
                          msg.linear_acceleration.y**2 +
                          msg.linear_acceleration.z**2)
        gyro_norm = np.sqrt(msg.angular_velocity.x**2 +
                           msg.angular_velocity.y**2 +
                           msg.angular_velocity.z**2)

        # Check for reasonable values (approximate Earth gravity for accelerometer)
        if abs(acc_norm - 9.81) > 2.0:  # More than 2 m/s² deviation from gravity
            rospy.logwarn("IMU accelerometer reading deviates significantly from expected gravity")

        if gyro_norm > 10.0:  # High angular velocity (adjust threshold as needed)
            rospy.logwarn("IMU gyroscope reading exceeds expected range")

        self.imu_stats['acc_norm'] = acc_norm
        self.imu_stats['gyro_norm'] = gyro_norm
        self.imu_stats['count'] += 1

    def publish_diagnostics(self, event):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = rospy.Time.now()

        # LiDAR diagnostic
        lidar_diag = DiagnosticStatus()
        lidar_diag.name = "LiDAR Sensor"
        lidar_diag.level = DiagnosticStatus.OK
        lidar_diag.message = "LiDAR sensor operational"

        if self.lidar_stats['count'] > 0:
            lidar_diag.values = [
                KeyValue(key="Min Range", value=str(self.lidar_stats['min_range'])),
                KeyValue(key="Max Range", value=str(self.lidar_stats['max_range'])),
                KeyValue(key="Mean Range", value=str(self.lidar_stats['mean_range'])),
                KeyValue(key="Sample Count", value=str(self.lidar_stats['count']))
            ]
        else:
            lidar_diag.level = DiagnosticStatus.WARN
            lidar_diag.message = "No LiDAR data received"

        # IMU diagnostic
        imu_diag = DiagnosticStatus()
        imu_diag.name = "IMU Sensor"
        imu_diag.level = DiagnosticStatus.OK
        imu_diag.message = "IMU sensor operational"

        if self.imu_stats['count'] > 0:
            imu_diag.values = [
                KeyValue(key="Acceleration Norm", value=str(self.imu_stats['acc_norm'])),
                KeyValue(key="Gyro Norm", value=str(self.imu_stats['gyro_norm'])),
                KeyValue(key="Sample Count", value=str(self.imu_stats['count']))
            ]
        else:
            imu_diag.level = DiagnosticStatus.WARN
            imu_diag.message = "No IMU data received"

        diag_array.status = [lidar_diag, imu_diag]
        self.diag_pub.publish(diag_array)

if __name__ == '__main__':
    rospy.init_node('sensor_validator')
    validator = SensorValidator()
    rospy.spin()
```

### Validation Criteria
- Diagnostic messages are published with sensor health information
- Sensor validation algorithms detect and report anomalies
- System provides clear feedback on sensor status
- Validation parameters are appropriately configured for each sensor type

## Exercise 6: Advanced Sensor Simulation Techniques

### Objective
Explore advanced techniques for improving sensor simulation realism.

### Tasks
1. Implement dynamic sensor noise models
2. Add environmental effects to sensor data
3. Create sensor degradation models

### Implementation Steps

1. Create a realistic sensor noise model:
```python
#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Imu
import random

class RealisticSensorModel:
    def __init__(self):
        # Initialize sensor-specific parameters
        self.lidar_bias = 0.0
        self.lidar_drift_rate = 0.001  # Bias drift per second
        self.lidar_temperature_coeff = 0.0001  # Per degree C
        self.lidar_temperature = 25.0  # Initial temperature in Celsius

        self.imu_acc_bias = np.array([0.0, 0.0, 0.0])
        self.imu_gyro_bias = np.array([0.0, 0.0, 0.0])

        # Subscribe to time for drift simulation
        self.last_update = rospy.Time.now()

        # Publishers for realistic sensor data
        self.real_lidar_pub = rospy.Publisher("/realistic_scan", LaserScan, queue_size=1)
        self.real_imu_pub = rospy.Publisher("/realistic_imu", Imu, queue_size=1)

    def update_temperature(self):
        # Simulate temperature changes (simplified)
        dt = 0.1  # Assuming 10Hz updates
        # Add some random walk to temperature
        temp_change = random.gauss(0, 0.1)
        self.lidar_temperature += temp_change
        # Keep within reasonable bounds
        self.lidar_temperature = max(10.0, min(40.0, self.lidar_temperature))

    def update_sensor_biases(self):
        # Update biases based on drift and temperature
        current_time = rospy.Time.now()
        dt = (current_time - self.last_update).to_sec()
        self.last_update = current_time

        if dt > 0:
            # Update LiDAR bias with drift and temperature effects
            self.lidar_bias += random.gauss(0, self.lidar_drift_rate * dt)
            self.lidar_bias += (self.lidar_temperature - 25.0) * self.lidar_temperature_coeff

    def add_realistic_noise(self, scan_msg):
        # Create a copy of the scan message
        noisy_scan = LaserScan()
        noisy_scan.header = scan_msg.header
        noisy_scan.angle_min = scan_msg.angle_min
        noisy_scan.angle_max = scan_msg.angle_max
        noisy_scan.angle_increment = scan_msg.angle_increment
        noisy_scan.time_increment = scan_msg.time_increment
        noisy_scan.scan_time = scan_msg.scan_time
        noisy_scan.range_min = scan_msg.range_min
        noisy_scan.range_max = scan_msg.range_max

        # Add realistic noise to ranges
        noisy_ranges = []
        for r in scan_msg.ranges:
            if r >= scan_msg.range_min and r <= scan_msg.range_max:
                # Range-dependent noise (typically increases with distance)
                noise_std = 0.01 + 0.001 * r  # 1cm + 0.1% of range
                noise = random.gauss(self.lidar_bias, noise_std)
                noisy_range = max(scan_msg.range_min, min(scan_msg.range_max, r + noise))
                noisy_ranges.append(noisy_range)
            else:
                noisy_ranges.append(r)  # Keep invalid ranges as is

        noisy_scan.ranges = noisy_ranges
        return noisy_scan

if __name__ == '__main__':
    rospy.init_node('realistic_sensor_model')
    model = RealisticSensorModel()
    rospy.spin()
```

### Validation Criteria
- Advanced noise models produce more realistic sensor data
- Environmental effects are properly simulated
- Sensor degradation is modeled over time
- System maintains realistic performance characteristics

## Assessment Questions

### Multiple Choice
1. What is the primary advantage of sensor fusion in robotics?
   A) Reduced computational requirements
   B) Improved accuracy and robustness
   C) Simplified sensor calibration
   D) Lower hardware costs

2. Which filter is most appropriate for nonlinear sensor fusion problems?
   A) Complementary filter
   B) Extended Kalman Filter
   C) Moving average filter
   D) Low-pass filter

### Short Answer
1. Explain the difference between data-level, feature-level, and decision-level fusion.
2. Describe how to handle temporal synchronization between sensors with different update rates.
3. What are the main challenges in fusing LiDAR and camera data?

### Practical Application
1. Design a sensor fusion system for a mobile robot that needs to navigate indoors using LiDAR, IMU, and wheel encoders.
2. Implement a validation algorithm that can detect when a sensor is providing unreliable data.

## Summary

These exercises provide hands-on experience with sensor simulation and fusion techniques. By completing these exercises, you will have gained practical experience in:
- Configuring and integrating multiple sensor types
- Processing sensor data in real-time
- Implementing sensor fusion algorithms
- Validating sensor performance
- Handling sensor failures and anomalies

The skills developed through these exercises are essential for creating robust robotic perception systems that can operate reliably in real-world environments.

---

**Next**: [Chapter 6 Summary](./summary.md) or [Sensor Simulation Introduction](./introduction.md)