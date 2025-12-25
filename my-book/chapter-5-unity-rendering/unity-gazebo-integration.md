---
title: Unity-Gazebo Integration
sidebar_label: Unity-Gazebo Integration
---

# Unity-Gazebo Integration

**Module**: Module 2 "The Digital Twin (Gazebo & Unity)"
**Chapter**: 5 - High-Fidelity Rendering and Human-Robot Interaction in Unity
**Section**: Unity-Gazebo Integration
**Created**: 2025-12-20
**Last Updated**: 2025-12-20

## Overview

Unity-Gazebo integration creates a comprehensive digital twin environment that combines Gazebo's physics simulation capabilities with Unity's high-fidelity rendering and human-robot interaction features. This integration enables the development of realistic, interactive simulation environments where the physics accuracy of Gazebo drives the visual realism of Unity, creating powerful tools for robotics research, development, and training.

## Integration Architecture

### System Overview

The Unity-Gazebo integration typically follows this architecture:

```
[ROS 2 Middleware]
       |
[Unity ROS TCP Connector] ↔ [Gazebo ROS Plugins]
       |                              |
[Unity Visualization]    [Gazebo Physics Simulation]
       |                              |
[Human Interaction]      [Robot Control & Sensors]
```

### Communication Protocols

#### ROS 2 Integration
- **DDS (Data Distribution Service)**: Underlying communication middleware
- **Topics**: Publish/subscribe communication for sensor data and commands
- **Services**: Request/response communication for specific operations
- **Actions**: Goal-based communication for long-running operations

#### TCP/IP Communication
- **Unity ROS TCP Connector**: Bridge between Unity and ROS 2
- **JSON Messages**: Human-readable message format
- **Custom Protocols**: Specialized communication for specific needs

## Unity ROS TCP Connector Setup

### Installation and Configuration

#### Prerequisites
- ROS 2 Humble Hawksbill installed
- Unity 2022.3 LTS or compatible version
- Python 3.8+ for ROS communication nodes

#### Installation Steps
1. **Download Unity ROS TCP Connector**:
   - Obtain from Unity Robotics GitHub repository
   - Import into Unity project as package

2. **Configure ROS Environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

3. **Launch ROS Bridge**:
   ```bash
   ros2 launch unity_robotics_demo unity_bridge.launch.py
   ```

### Basic Connection Setup
```csharp
// Example Unity script for ROS connection
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class RobotConnectionManager : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    private ROSConnection rosConnection;

    void Start()
    {
        // Connect to ROS
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosConnection.Connect(rosIPAddress, rosPort);

        // Subscribe to robot state topics
        SubscribeToRobotTopics();
    }

    void SubscribeToRobotTopics()
    {
        // Subscribe to joint states
        rosConnection.Subscribe<Unity.Robotics.ROSMessageTypes.Sensor.JointStateMsg>(
            "/joint_states", OnJointStateReceived
        );

        // Subscribe to robot transforms
        rosConnection.Subscribe<Unity.Robotics.ROSMessageTypes.Geometry.PoseStampedMsg>(
            "/robot_pose", OnRobotPoseReceived
        );

        // Subscribe to sensor data
        rosConnection.Subscribe<Unity.Robotics.ROSMessageTypes.Sensor.LaserScanMsg>(
            "/scan", OnLaserScanReceived
        );
    }

    void OnJointStateReceived(Unity.Robotics.ROSMessageTypes.Sensor.JointStateMsg jointState)
    {
        // Process joint state message and update Unity visualization
        UpdateRobotVisualization(jointState);
    }

    void OnRobotPoseReceived(Unity.Robotics.ROSMessageTypes.Geometry.PoseStampedMsg pose)
    {
        // Update robot position and orientation in Unity
        UpdateRobotPose(pose);
    }

    void OnLaserScanReceived(Unity.Robotics.ROSMessageTypes.Sensor.LaserScanMsg scan)
    {
        // Process LiDAR data for visualization
        UpdateLidarVisualization(scan);
    }

    void UpdateRobotVisualization(Unity.Robotics.ROSMessageTypes.Sensor.JointStateMsg jointState)
    {
        // Update robot joint positions based on Gazebo simulation
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            float jointPosition = jointState.position[i];

            Transform jointTransform = FindJointByName(jointName);
            if (jointTransform != null)
            {
                // Apply joint position to Unity transform
                jointTransform.localRotation =
                    Quaternion.Euler(0, jointPosition * Mathf.Rad2Deg, 0);
            }
        }
    }

    Transform FindJointByName(string name)
    {
        // Find joint transform by name in robot hierarchy
        Transform[] allChildren = GetComponentsInChildren<Transform>();
        foreach (Transform child in allChildren)
        {
            if (child.name == name)
                return child;
        }
        return null;
    }

    void UpdateRobotPose(Unity.Robotics.ROSMessageTypes.Geometry.PoseStampedMsg pose)
    {
        // Update robot's base position and orientation
        transform.position = new Vector3(
            (float)pose.pose.position.x,
            (float)pose.pose.position.z,  // Note: coordinate conversion
            (float)pose.pose.position.y
        );

        transform.rotation = new Quaternion(
            (float)pose.pose.orientation.x,
            (float)pose.pose.orientation.z,
            (float)pose.pose.orientation.y,
            (float)pose.pose.orientation.w
        );
    }

    void UpdateLidarVisualization(Unity.Robotics.ROSMessageTypes.Sensor.LaserScanMsg scan)
    {
        // Update LiDAR visualization based on scan data
        // Implementation depends on specific visualization needs
    }
}
```

## Data Synchronization

### Coordinate System Conversion

Unity uses a left-handed coordinate system while ROS/Gazebo uses right-handed:

```csharp
// Coordinate system conversion utilities
public static class CoordinateConverter
{
    public static Vector3 RosToUnity(Vector3 rosVector)
    {
        // Convert from ROS (right-handed) to Unity (left-handed)
        // ROS: X-forward, Y-left, Z-up
        // Unity: X-right, Y-up, Z-forward
        return new Vector3(
            rosVector.x,    // X remains X
            rosVector.z,    // Z becomes Y (up)
            -rosVector.y    // -Y becomes Z (forward)
        );
    }

    public static Vector3 UnityToRos(Vector3 unityVector)
    {
        // Convert from Unity (left-handed) to ROS (right-handed)
        return new Vector3(
            unityVector.x,      // X remains X
            -unityVector.z,     // -Z becomes Y
            unityVector.y       // Y becomes Z
        );
    }

    public static Quaternion RosToUnity(Quaternion rosQuaternion)
    {
        // Convert ROS quaternion to Unity quaternion
        return new Quaternion(
            rosQuaternion.x,
            rosQuaternion.z,
            -rosQuaternion.y,
            rosQuaternion.w
        );
    }

    public static Quaternion UnityToRos(Quaternion unityQuaternion)
    {
        // Convert Unity quaternion to ROS quaternion
        return new Quaternion(
            unityQuaternion.x,
            -unityQuaternion.z,
            unityQuaternion.y,
            unityQuaternion.w
        );
    }
}
```

### Time Synchronization

Ensure Unity and Gazebo clocks stay synchronized:

```csharp
// Time synchronization between Unity and Gazebo
public class TimeSynchronizer : MonoBehaviour
{
    [Header("Time Settings")]
    public float simulationTimeScale = 1.0f;
    public bool useRealTime = true;

    private double gazeboTime = 0.0;
    private double unityTime = 0.0;

    void Update()
    {
        if (useRealTime)
        {
            // Synchronize with Gazebo time if needed
            SynchronizeTime();
        }
    }

    public void OnGazeboTimeReceived(double time)
    {
        gazeboTime = time;
    }

    void SynchronizeTime()
    {
        // Adjust Unity time scale to match Gazebo
        if (gazeboTime > 0)
        {
            float timeDiff = (float)(gazeboTime - unityTime);
            if (Mathf.Abs(timeDiff) > 0.1f)
            {
                // Adjust time scale to reduce time difference
                Time.timeScale = Mathf.Clamp(1.0f + timeDiff * 0.1f, 0.1f, 2.0f);
            }
        }
    }
}
```

## Robot State Visualization

### Joint State Synchronization

Synchronize robot joint states between Gazebo and Unity:

```csharp
// Joint state synchronization
using System.Collections.Generic;
using Unity.Robotics.ROSMessageTypes.Sensor;

public class JointStateSynchronizer : MonoBehaviour
{
    [Header("Robot Configuration")]
    public string robotName = "basic_humanoid";
    public string jointStateTopic = "/joint_states";

    [Header("Joint Mapping")]
    public JointMapping[] jointMappings;

    private Dictionary<string, Transform> jointTransforms;
    private ROSConnection rosConnection;

    [System.Serializable]
    public class JointMapping
    {
        public string gazeboJointName;
        public string unityJointName;
        public JointType jointType;
        public float positionMultiplier = 1.0f;
        public float minAngle = -180f;
        public float maxAngle = 180f;
    }

    public enum JointType
    {
        Revolute,
        Prismatic,
        Fixed
    }

    void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        InitializeJointTransforms();
        SubscribeToJointStates();
    }

    void InitializeJointTransforms()
    {
        jointTransforms = new Dictionary<string, Transform>();

        foreach (JointMapping mapping in jointMappings)
        {
            Transform jointTransform = FindTransformByName(mapping.unityJointName);
            if (jointTransform != null)
            {
                jointTransforms[mapping.gazeboJointName] = jointTransform;
            }
        }
    }

    void SubscribeToJointStates()
    {
        rosConnection.Subscribe<JointStateMsg>(jointStateTopic, OnJointStateReceived);
    }

    void OnJointStateReceived(JointStateMsg jointState)
    {
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            double jointPosition = jointState.position[i];

            if (jointTransforms.ContainsKey(jointName))
            {
                Transform jointTransform = jointTransforms[jointName];
                JointMapping mapping = GetJointMapping(jointName);

                ApplyJointPosition(jointTransform, mapping, (float)jointPosition);
            }
        }
    }

    JointMapping GetJointMapping(string jointName)
    {
        foreach (JointMapping mapping in jointMappings)
        {
            if (mapping.gazeboJointName == jointName)
                return mapping;
        }
        return null;
    }

    void ApplyJointPosition(Transform jointTransform, JointMapping mapping, float position)
    {
        float clampedPosition = Mathf.Clamp(position * mapping.positionMultiplier,
                                          mapping.minAngle, mapping.maxAngle);

        switch (mapping.jointType)
        {
            case JointType.Revolute:
                jointTransform.localRotation =
                    Quaternion.Euler(0, clampedPosition * Mathf.Rad2Deg, 0);
                break;
            case JointType.Prismatic:
                jointTransform.localPosition =
                    jointTransform.localPosition +
                    jointTransform.right * clampedPosition;
                break;
            case JointType.Fixed:
                // Fixed joints don't move
                break;
        }
    }

    Transform FindTransformByName(string name)
    {
        Transform[] allChildren = GetComponentsInChildren<Transform>();
        foreach (Transform child in allChildren)
        {
            if (child.name == name)
                return child;
        }
        return null;
    }
}
```

### Sensor Data Integration

Integrate sensor data from Gazebo into Unity visualization:

```csharp
// Sensor data integration
using Unity.Robotics.ROSMessageTypes.Sensor;

public class SensorDataIntegrator : MonoBehaviour
{
    [Header("Sensor Topics")]
    public string laserScanTopic = "/scan";
    public string imageTopic = "/camera/image_raw";
    public string imuTopic = "/imu/data";

    [Header("Visualization Components")]
    public GameObject lidarPointCloud;
    public GameObject cameraDisplay;
    public GameObject imuVisualizer;

    private ROSConnection rosConnection;

    void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        SubscribeToSensors();
    }

    void SubscribeToSensors()
    {
        rosConnection.Subscribe<LaserScanMsg>(laserScanTopic, OnLaserScanReceived);
        rosConnection.Subscribe<ImageMsg>(imageTopic, OnImageReceived);
        rosConnection.Subscribe<ImuMsg>(imuTopic, OnImuReceived);
    }

    void OnLaserScanReceived(LaserScanMsg scan)
    {
        // Process LiDAR scan data
        float[] ranges = new float[scan.ranges.Count];
        for (int i = 0; i < scan.ranges.Count; i++)
        {
            ranges[i] = (float)scan.ranges[i];
        }

        UpdateLidarVisualization(ranges, scan.angle_min, scan.angle_increment);
    }

    void OnImageReceived(ImageMsg image)
    {
        // Process camera image data
        UpdateCameraDisplay(image);
    }

    void OnImuReceived(ImuMsg imu)
    {
        // Process IMU data
        UpdateImuVisualization(imu);
    }

    void UpdateLidarVisualization(float[] ranges, double angleMin, double angleIncrement)
    {
        // Update LiDAR point cloud visualization
        if (lidarPointCloud != null)
        {
            LidarVisualizer lidarVis = lidarPointCloud.GetComponent<LidarVisualizer>();
            if (lidarVis != null)
            {
                // Convert angles to array for visualization
                float[] angles = new float[ranges.Length];
                for (int i = 0; i < ranges.Length; i++)
                {
                    angles[i] = (float)(angleMin + i * angleIncrement);
                }

                lidarVis.UpdateLidarData(ranges, angles);
            }
        }
    }

    void UpdateCameraDisplay(ImageMsg image)
    {
        // Update camera display with image data
        if (cameraDisplay != null)
        {
            CameraDisplay camDisplay = cameraDisplay.GetComponent<CameraDisplay>();
            if (camDisplay != null)
            {
                camDisplay.UpdateImage(image);
            }
        }
    }

    void UpdateImuVisualization(ImuMsg imu)
    {
        // Update IMU visualization
        if (imuVisualizer != null)
        {
            ImuVisualizer imuVis = imuVisualizer.GetComponent<ImuVisualizer>();
            if (imuVis != null)
            {
                imuVis.UpdateOrientation(imu.orientation);
            }
        }
    }
}
```

## Control Integration

### Sending Commands to Gazebo

Send control commands from Unity to Gazebo:

```csharp
// Control command sender
using Unity.Robotics.ROSMessageTypes.Std;
using Unity.Robotics.ROSMessageTypes.Geometry;

public class ControlCommandSender : MonoBehaviour
{
    [Header("Control Topics")]
    public string jointCommandTopic = "/joint_group_position_controller/commands";
    public string velocityCommandTopic = "/cmd_vel";

    private ROSConnection rosConnection;

    void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
    }

    public void SendJointCommands(Dictionary<string, float> jointCommands)
    {
        // Send joint position commands to Gazebo
        var jointCmd = new Unity.Robotics.ROSMessageTypes.Sensor.JointStateMsg();

        foreach (var kvp in jointCommands)
        {
            jointCmd.name.Add(kvp.Key);
            jointCmd.position.Add(kvp.Value);
        }

        rosConnection.Publish(jointCommandTopic, jointCmd);
    }

    public void SendVelocityCommand(float linearX, float angularZ)
    {
        // Send velocity commands to Gazebo
        var twist = new TwistMsg();
        twist.linear = new Vector3Msg(linearX, 0, 0);
        twist.angular = new Vector3Msg(0, 0, angularZ);

        rosConnection.Publish(velocityCommandTopic, twist);
    }

    public void SendNavigationGoal(Vector3 targetPosition)
    {
        // Send navigation goal to ROS navigation stack
        var goal = new Unity.Robotics.ROSMessageTypes.Geometry.PoseStampedMsg();
        goal.header = new HeaderMsg();
        goal.header.stamp = new TimeMsg();
        goal.header.frame_id = "map";

        goal.pose.position = new Vector3Msg(targetPosition.x, targetPosition.y, targetPosition.z);
        goal.pose.orientation = new QuaternionMsg(0, 0, 0, 1); // Identity orientation

        rosConnection.Publish("/move_base_simple/goal", goal);
    }
}
```

## Performance Optimization

### Data Transmission Optimization

Optimize data transmission between Unity and Gazebo:

```csharp
// Optimized data transmission
public class OptimizedDataTransmitter : MonoBehaviour
{
    [Header("Optimization Settings")]
    public float updateRate = 30.0f; // Hz
    public bool compressData = true;
    public float positionThreshold = 0.01f;
    public float rotationThreshold = 0.1f; // degrees

    private float lastUpdateTime = 0f;
    private Vector3 lastPosition = Vector3.zero;
    private Quaternion lastRotation = Quaternion.identity;

    void Update()
    {
        if (Time.time - lastUpdateTime >= 1.0f / updateRate)
        {
            TransmitOptimizedData();
            lastUpdateTime = Time.time;
        }
    }

    void TransmitOptimizedData()
    {
        // Only transmit if significant change occurred
        if (Vector3.Distance(transform.position, lastPosition) > positionThreshold ||
            Quaternion.Angle(transform.rotation, lastRotation) > rotationThreshold)
        {
            SendTransformUpdate();
            lastPosition = transform.position;
            lastRotation = transform.rotation;
        }
    }

    void SendTransformUpdate()
    {
        // Send optimized transform data to Gazebo
        var pose = new Unity.Robotics.ROSMessageTypes.Geometry.PoseStampedMsg();
        pose.header = new Unity.Robotics.ROSMessageTypes.Std.HeaderMsg();
        pose.header.stamp = new Unity.Robotics.ROSMessageTypes.Std.TimeMsg();
        pose.header.frame_id = "world";

        var unityPos = CoordinateConverter.UnityToRos(transform.position);
        var unityRot = CoordinateConverter.UnityToRos(transform.rotation);

        pose.pose.position = new Unity.Robotics.ROSMessageTypes.Geometry.Vector3Msg(
            unityPos.x, unityPos.y, unityPos.z
        );
        pose.pose.orientation = new Unity.Robotics.ROSMessageTypes.Geometry.QuaternionMsg(
            unityRot.x, unityRot.y, unityRot.z, unityRot.w
        );

        // Publish with optimization
        ROSConnection.GetOrCreateInstance().Publish("/unity_robot_pose", pose);
    }
}
```

### Message Batching

Batch multiple messages for efficient transmission:

```csharp
// Message batching system
using System.Collections.Generic;

public class MessageBatcher : MonoBehaviour
{
    [Header("Batch Settings")]
    public int maxBatchSize = 10;
    public float batchInterval = 0.1f; // seconds

    private List<BatchMessage> pendingMessages = new List<BatchMessage>();
    private float lastBatchTime = 0f;

    [System.Serializable]
    public class BatchMessage
    {
        public string topic;
        public object message;
    }

    void Update()
    {
        if (Time.time - lastBatchTime >= batchInterval)
        {
            SendBatch();
            lastBatchTime = Time.time;
        }
    }

    public void AddToBatch(string topic, object message)
    {
        pendingMessages.Add(new BatchMessage { topic = topic, message = message });

        if (pendingMessages.Count >= maxBatchSize)
        {
            SendBatch();
        }
    }

    void SendBatch()
    {
        if (pendingMessages.Count == 0) return;

        ROSConnection ros = ROSConnection.GetOrCreateInstance();

        foreach (var msg in pendingMessages)
        {
            ros.Publish(msg.topic, msg.message);
        }

        pendingMessages.Clear();
    }
}
```

## Error Handling and Robustness

### Connection Management

Handle connection issues gracefully:

```csharp
// Connection management with error handling
public class RobustConnectionManager : MonoBehaviour
{
    [Header("Connection Settings")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;
    public float reconnectInterval = 5.0f;
    public int maxReconnectAttempts = 10;

    private ROSConnection rosConnection;
    private float lastConnectionAttempt = 0f;
    private int reconnectAttempts = 0;
    private bool isConnected = false;

    void Start()
    {
        InitializeConnection();
    }

    void InitializeConnection()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        AttemptConnection();
    }

    void AttemptConnection()
    {
        try
        {
            rosConnection.Connect(rosIPAddress, rosPort);
            isConnected = true;
            reconnectAttempts = 0;
            Debug.Log("Successfully connected to ROS bridge");

            // Re-subscribe to topics after reconnection
            SubscribeToTopics();
        }
        catch (System.Exception e)
        {
            isConnected = false;
            Debug.LogError($"Failed to connect to ROS: {e.Message}");

            if (reconnectAttempts < maxReconnectAttempts)
            {
                reconnectAttempts++;
                lastConnectionAttempt = Time.time;
            }
            else
            {
                Debug.LogError("Max reconnection attempts reached. Please check ROS bridge.");
            }
        }
    }

    void Update()
    {
        // Handle reconnection attempts
        if (!isConnected && Time.time - lastConnectionAttempt >= reconnectInterval)
        {
            AttemptConnection();
        }
    }

    void SubscribeToTopics()
    {
        // Re-subscribe to all necessary topics
        rosConnection.Subscribe<Unity.Robotics.ROSMessageTypes.Sensor.JointStateMsg>(
            "/joint_states", OnJointStateReceived
        );
        // Add other subscriptions as needed
    }

    void OnJointStateReceived(Unity.Robotics.ROSMessageTypes.Sensor.JointStateMsg jointState)
    {
        if (isConnected)
        {
            UpdateRobotVisualization(jointState);
        }
    }

    void UpdateRobotVisualization(Unity.Robotics.ROSMessageTypes.Sensor.JointStateMsg jointState)
    {
        // Update visualization logic here
    }

    void OnApplicationQuit()
    {
        // Clean up connection
        if (rosConnection != null)
        {
            rosConnection.Close();
        }
    }
}
```

## Best Practices for Integration

### Architecture Guidelines

1. **Separation of Concerns**: Keep physics simulation logic separate from visualization logic
2. **Message Efficiency**: Minimize message size and frequency
3. **Error Handling**: Implement robust error handling and recovery
4. **Performance Monitoring**: Monitor and optimize communication performance

### Data Flow Patterns

1. **State-Driven**: Unity visualization updates based on Gazebo state
2. **Event-Driven**: Specific events trigger data transmission
3. **Predictive**: Use interpolation to smooth out network delays
4. **Buffered**: Use buffers to handle variable message timing

### Testing and Validation

1. **Unit Testing**: Test individual components independently
2. **Integration Testing**: Test Unity-Gazebo communication
3. **Performance Testing**: Validate real-time performance
4. **Robustness Testing**: Test under various network conditions

## Common Integration Challenges

### Synchronization Issues
- **Time synchronization**: Ensure Unity and Gazebo clocks stay aligned
- **State synchronization**: Keep robot states consistent between systems
- **Frame rate differences**: Handle different update rates gracefully

### Performance Bottlenecks
- **Network bandwidth**: Optimize message size and frequency
- **Processing overhead**: Minimize computational load on both systems
- **Memory usage**: Efficiently manage data structures and buffers

### Coordinate System Conflicts
- **Unit differences**: Handle meter vs. Unity unit scaling
- **Axis orientation**: Convert between different coordinate systems
- **Rotation conventions**: Convert between different rotation representations

## Troubleshooting Common Issues

### Connection Problems
- **Port conflicts**: Ensure ROS bridge ports are available
- **Firewall issues**: Configure firewall for ROS communication
- **Network configuration**: Verify IP addresses and network settings

### Data Synchronization
- **Message loss**: Implement message acknowledgment if needed
- **Timing issues**: Use appropriate buffer sizes and update rates
- **Coordinate conversion**: Double-check conversion formulas

### Performance Issues
- **Frame rate drops**: Optimize rendering and reduce update frequency
- **Memory leaks**: Monitor and fix resource management issues
- **CPU usage**: Profile and optimize computational bottlenecks

## Summary

Unity-Gazebo integration creates a powerful digital twin environment that combines the physics accuracy of Gazebo with the visual capabilities of Unity. Success in this integration requires careful attention to data synchronization, coordinate system conversion, performance optimization, and robust error handling. By following the patterns and best practices outlined in this section, you can create effective, stable, and performant Unity-Gazebo integration systems that serve as valuable tools for robotics development and research.

The key to successful integration lies in understanding the strengths of each system and designing communication protocols that leverage both physics simulation and high-fidelity visualization capabilities effectively.

---

**Next**: [Unity Exercises](./exercises.md)