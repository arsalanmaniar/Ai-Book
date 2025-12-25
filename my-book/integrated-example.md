---
title: Integrated Simulation Example
sidebar_label: Integrated Example
---

# Integrated Simulation Example: Complete Digital Twin Implementation

**Module**: Module 2 "The Digital Twin (Gazebo & Unity)"
**Document**: Integrated Simulation Example
**Created**: 2025-12-20
**Last Updated**: 2025-12-20

## Overview

This example demonstrates a complete integrated simulation system that combines Gazebo physics, Unity rendering, and sensor simulation into a unified digital twin environment. The example implements a humanoid robot performing navigation and manipulation tasks with realistic physics, high-fidelity rendering, and comprehensive sensor simulation.

## System Architecture

### Components Overview

The integrated system consists of three main components:

1. **Gazebo Physics Engine**: Handles realistic physics simulation including rigid body dynamics, collision detection, and contact mechanics
2. **Unity Visualization**: Provides high-fidelity rendering and human-robot interaction interfaces
3. **Sensor Simulation**: Simulates various sensors including LiDAR, cameras, and IMUs
4. **Integration Layer**: Synchronizes data between all components

### Data Flow Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Gazebo        │    │   Integration    │    │    Unity        │
│   Physics       │◄──►│   Layer (ROS)    │◄──►│   Rendering     │
│   - Dynamics    │    │   - State Sync   │    │   - PBR         │
│   - Collision   │    │   - TF Transform │    │   - Lighting    │
│   - Contacts    │    │   - Message      │    │   - HRI         │
└─────────────────┘    │   Bridge         │    └─────────────────┘
                       └──────────────────┘
                                ▲
                                │
                       ┌──────────────────┐
                       │   Sensor         │
                       │   Simulation     │
                       │   - LiDAR        │
                       │   - Camera       │
                       │   - IMU          │
                       │   - Fusion       │
                       └──────────────────┘
```

## Implementation Example

### 1. Robot Model Definition (URDF)

First, we define a complete humanoid robot model with all necessary sensors:

```xml
<?xml version="1.0"?>
<robot name="integrated_humanoid">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.15" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.15" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <joint name="base_torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- Head with Sensors -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="torso_head_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
  </joint>

  <!-- LiDAR Sensor -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="head_lidar_joint" type="fixed">
    <parent link="head"/>
    <child link="lidar_link"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Camera Sensor -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.04 0.15 0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.04 0.15 0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="head_camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.06 0 0.02" rpy="0 0 0"/>
  </joint>

  <!-- IMU Sensor -->
  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
      <material name="green">
        <color rgba="0.0 1.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="torso_imu_joint" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo Plugins for Physics -->
  <gazebo reference="base_link">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="torso">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
    <material>Gazebo/Gray</material>
  </gazebo>

  <gazebo reference="head">
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
    <material>Gazebo/White</material>
  </gazebo>

  <!-- LiDAR Sensor Plugin -->
  <gazebo reference="lidar_link">
    <sensor type="ray" name="head_lidar">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
        <topicName>scan</topicName>
        <frameName>lidar_link</frameName>
        <min_range>0.1</min_range>
        <max_range>30.0</max_range>
        <gaussianNoise>0.01</gaussianNoise>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Camera Sensor Plugin -->
  <gazebo reference="camera_link">
    <sensor type="depth" name="head_camera">
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

  <!-- IMU Sensor Plugin -->
  <gazebo reference="imu_link">
    <sensor type="imu" name="torso_imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
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

  <!-- Joint State Publisher -->
  <gazebo>
    <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
      <robotNamespace>/</robotNamespace>
      <jointName>torso_head_joint</jointName>
    </plugin>
  </gazebo>
</robot>
```

### 2. Gazebo World Configuration

Create a world file that includes physics properties and environment:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="integrated_demo_world">
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics properties -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Environment objects -->
    <model name="table">
      <pose>2 1 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 0.8 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.4 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 0.8 1</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>10</mass>
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

    <model name="obstacle_1">
      <pose>-1 0 0.2 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.4</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.4 0.8 0.4 1</ambient>
            <diffuse>0.4 0.8 0.4 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.4</length>
            </cylinder>
          </geometry>
        </collision>
        <inertial>
          <mass>2</mass>
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

    <model name="obstacle_2">
      <pose>0 -2 0.1 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>0.3 0.3 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.4 0.4 1</ambient>
            <diffuse>0.8 0.4 0.4 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.3 0.3 0.2</size>
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

### 3. Unity Scene Configuration

Create a Unity scene that mirrors the Gazebo environment. This is a conceptual representation of what would be in the .unity file:

```
%YAML 1.1
%TAG !u! tag:unity3d.com,2011:
--- !u!29 &1
OcclusionCullingSettings:
  m_ObjectHideFlags: 0
  serializedVersion: 2
  m_OcclusionBakeSettings:
    smallestOccluder: 5
    smallestHole: 0.25
    backfaceThreshold: 100
  m_SceneGUID: 00000000000000000000000000000000
  m_OcclusionCullingData: {fileID: 0}
--- !u!104 &2
RenderSettings:
  m_ObjectHideFlags: 0
  serializedVersion: 9
  m_Fog: 0
  m_FogColor: {r: 0.5, g: 0.5, b: 0.5, a: 1}
  m_FogMode: 3
  m_FogDensity: 0.01
  m_LinearFogStart: 0
  m_LinearFogEnd: 300
  m_AmbientSkyColor: {r: 0.212, g: 0.227, b: 0.259, a: 1}
  m_AmbientEquatorColor: {r: 0.114, g: 0.125, b: 0.133, a: 1}
  m_AmbientGroundColor: {r: 0.047, g: 0.043, b: 0.035, a: 1}
  m_AmbientIntensity: 1
  m_AmbientMode: 0
  m_SubtractiveShadowColor: {r: 0.42, g: 0.478, b: 0.627, a: 1}
  m_SkyboxMaterial: {fileID: 10304, guid: 0000000000000000f000000000000000, type: 0}
  m_HaloStrength: 0.5
  m_FlareStrength: 1
  m_FlareFadeSpeed: 3
  m_HaloTexture: {fileID: 0}
  m_SpotCookie: {fileID: 10001, guid: 0000000000000000e000000000000000, type: 0}
  m_DefaultReflectionMode: 0
  m_DefaultReflectionResolution: 128
  m_ReflectionBounces: 1
  m_ReflectionIntensity: 1
  m_CustomReflection: {fileID: 0}
  m_Sun: {fileID: 0}
  m_IndirectSpecularColor: {r: 0.18028378, g: 0.22571412, b: 0.30692285, a: 1}
  m_UseRadianceAmbientProbe: 0
--- !u!157 &3
LightmapSettings:
  m_ObjectHideFlags: 0
  serializedVersion: 12
  m_GIWorkflowMode: 1
  m_GISettings:
    serializedVersion: 2
    m_BounceScale: 1
    m_IndirectOutputScale: 1
    m_AlbedoBoost: 1
    m_EnvironmentLightingMode: 0
    m_EnableBakedLightmaps: 1
    m_EnableRealtimeLightmaps: 0
  m_LightmapEditorSettings:
    serializedVersion: 12
    m_Resolution: 2
    m_BakeResolution: 40
    m_AtlasSize: 1024
    m_AO: 0
    m_AOMaxDistance: 1
    m_CompAOExponent: 1
    m_CompAOExponentDirect: 0
    m_ExtractAmbientOcclusion: 0
    m_Padding: 2
    m_LightmapParameters: {fileID: 0}
    m_LightmapsBakeMode: 1
    m_TextureCompression: 1
    m_FinalGather: 0
    m_FinalGatherFiltering: 1
    m_FinalGatherRayCount: 256
    m_ReflectionCompression: 2
    m_MixedBakeMode: 2
    m_BakeBackend: 1
    m_PVRSampling: 1
    m_PVRDirectSampleCount: 32
    m_PVRSampleCount: 512
    m_PVRBounces: 2
    m_PVREnvironmentSampleCount: 256
    m_PVREnvironmentReferencePointCount: 2048
    m_PVRFilteringMode: 1
    m_PVRDenoiserTypeDirect: 1
    m_PVRDenoiserTypeIndirect: 1
    m_PVRDenoiserTypeAO: 1
    m_PVRFilterTypeDirect: 0
    m_PVRFilterTypeIndirect: 0
    m_PVRFilterTypeAO: 0
    m_PVREnvironmentMIS: 1
    m_PVRCulling: 1
    m_PVRFilteringGaussRadiusDirect: 1
    m_PVRFilteringGaussRadiusIndirect: 5
    m_PVRFilteringGaussRadiusAO: 2
    m_PVRFilteringAtrousPositionSigmaDirect: 0.5
    m_PVRFilteringAtrousPositionSigmaIndirect: 2
    m_PVRFilteringAtrousPositionSigmaAO: 1
    m_ExportTrainingData: 0
    m_TrainingDataDestination: TrainingData
    m_LightProbeSampleCountMultiplier: 4
  m_LightingDataAsset: {fileID: 0}
  m_LightingSettings: {fileID: 0}
--- !u!196 &4
NavMeshSettings:
  serializedVersion: 2
  m_ObjectHideFlags: 0
  m_BuildSettings:
    serializedVersion: 2
    agentTypeID: 0
    agentRadius: 0.5
    agentHeight: 2
    agentSlope: 45
    agentClimb: 0.4
    ledgeDropHeight: 0
    maxJumpAcrossDistance: 0
    minRegionArea: 2
    manualCellSize: 0
    cellSize: 0.16666667
    manualTileSize: 0
    tileSize: 256
    accuratePlacement: 0
    maxJobWorkers: 0
    preserveTilesOutsideBounds: 0
    debug:
      m_Flags: 0
  m_NavMeshData: {fileID: 0}
--- !u!1 &100000
GameObject:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {fileID: 0}
  m_PrefabInstance: {fileID: 0}
  m_PrefabAsset: {fileID: 0}
  serializedVersion: 6
  m_Component:
  - component: {fileID: 100002}
  - component: {fileID: 100001}
  m_Layer: 0
  m_Name: Main Camera
  m_TagString: MainCamera
  m_Icon: {fileID: 0}
  m_NavMeshLayer: 0
  m_StaticEditorFlags: 0
  m_IsActive: 1
--- !u!20 &100001
Camera:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {fileID: 0}
  m_PrefabInstance: {fileID: 0}
  m_PrefabAsset: {fileID: 0}
  m_GameObject: {fileID: 100000}
  m_Enabled: 1
  serializedVersion: 2
  m_ClearFlags: 1
  m_BackGroundColor: {r: 0.19215687, g: 0.3019608, b: 0.4745098, a: 0}
  m_projectionMatrixMode: 1
  m_GateFitMode: 2
  m_FOVAxisMode: 0
  m_SensorSize: {x: 36, y: 24}
  m_LensShift: {x: 0, y: 0}
  m_FocalLength: 50
  m_NormalizedViewPortRect:
    serializedVersion: 2
    x: 0
    y: 0
    width: 1
    height: 1
  near clip plane: 0.3
  far clip plane: 1000
  field of view: 60
  orthographic: 0
  orthographic size: 5
  m_Depth: -1
  m_CullingMask:
    serializedVersion: 2
    m_Bits: 4294967295
  m_RenderingPath: -1
  m_TargetTexture: {fileID: 0}
  m_TargetDisplay: 0
  m_TargetEye: 3
  m_HDR: 1
  m_AllowMSAA: 1
  m_AllowDynamicResolution: 0
  m_ForceIntoRT: 0
  m_OcclusionCulling: 1
  m_StereoConvergence: 10
  m_StereoSeparation: 0.022
--- !u!4 &100002
Transform:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {fileID: 0}
  m_PrefabInstance: {fileID: 0}
  m_PrefabAsset: {fileID: 0}
  m_GameObject: {fileID: 100000}
  m_LocalRotation: {x: 0, y: 0, z: 0, w: 1}
  m_LocalPosition: {x: 0, y: 1, z: -10}
  m_LocalScale: {x: 1, y: 1, z: 1}
  m_ConstrainProportionsScale: 0
  m_Children: []
  m_Father: {fileID: 0}
  m_RootOrder: 0
  m_LocalEulerAnglesHint: {x: 0, y: 0, z: 0}
--- !u!1 &100004
GameObject:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {fileID: 0}
  m_PrefabInstance: {fileID: 0}
  m_PrefabAsset: {fileID: 0}
  serializedVersion: 6
  m_Component:
  - component: {fileID: 100006}
  - component: {fileID: 100005}
  m_Layer: 0
  m_Name: Directional Light
  m_TagString: Untagged
  m_Icon: {fileID: 0}
  m_NavMeshLayer: 0
  m_StaticEditorFlags: 0
  m_IsActive: 1
--- !u!108 &100005
Light:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {fileID: 0}
  m_PrefabInstance: {fileID: 0}
  m_PrefabAsset: {fileID: 0}
  m_GameObject: {fileID: 100004}
  m_Enabled: 1
  serializedVersion: 10
  m_Type: 1
  m_Shape: 0
  m_Color: {r: 1, g: 0.95686275, b: 0.8392157, a: 1}
  m_Intensity: 1
  m_Range: 10
  m_SpotAngle: 30
  m_InnerSpotAngle: 21.80208
  m_CookieSize: 10
  m_Shadows:
    m_Type: 2
    m_Resolution: -1
    m_CustomResolution: -1
    m_Strength: 1
    m_Bias: 0.05
    m_NormalBias: 0.4
    m_NearPlane: 0.2
    m_CullingMatrixOverride:
      e00: 1
      e01: 0
      e02: 0
      e03: 0
      e10: 0
      e11: 1
      e12: 0
      e13: 0
      e20: 0
      e21: 0
      e22: 1
      e23: 0
      e30: 0
      e31: 0
      e32: 0
      e33: 1
    m_UseCullingMatrixOverride: 0
  m_Cookie: {fileID: 0}
  m_DrawHalo: 0
  m_Flare: {fileID: 0}
  m_RenderMode: 0
  m_CullingMask:
    serializedVersion: 2
    m_Bits: 4294967295
  m_RenderingLayerMask: 1
  m_Lightmapping: 4
  m_LightShadowCasterMode: 0
  m_AreaSize: {x: 1, y: 1}
  m_BounceIntensity: 1
  m_ColorTemperature: 6570
  m_UseColorTemperature: 0
  m_BoundingSphereOverride: {x: 0, y: 0, z: 0, w: 0}
  m_UseBoundingSphereOverride: 0
  m_UseViewFrustumForShadowCascades: 1
  m_ShadowRadius: 0
  m_ShadowAngle: 0
--- !u!4 &100006
Transform:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {fileID: 0}
  m_PrefabInstance: {fileID: 0}
  m_PrefabAsset: {fileID: 0}
  m_GameObject: {fileID: 100004}
  m_LocalRotation: {x: 0.40821788, y: -0.23456968, z: 0.10938163, w: 0.8754261}
  m_LocalPosition: {x: 0, y: 3, z: 0}
  m_LocalScale: {x: 1, y: 1, z: 1}
  m_ConstrainProportionsScale: 0
  m_Children: []
  m_Father: {fileID: 0}
  m_RootOrder: 1
  m_LocalEulerAnglesHint: {x: 50, y: -30, z: 0}
--- !u!1 &500000
GameObject:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {fileID: 0}
  m_PrefabInstance: {fileID: 0}
  m_PrefabAsset: {fileID: 0}
  serializedVersion: 6
  m_Component:
  - component: {fileID: 500004}
  - component: {fileID: 500003}
  - component: {fileID: 500002}
  - component: {fileID: 500001}
  m_Layer: 0
  m_Name: HumanoidRobot
  m_TagString: Untagged
  m_Icon: {fileID: 0}
  m_NavMeshLayer: 0
  m_StaticEditorFlags: 0
  m_IsActive: 1
--- !u!114 &500001
MonoBehaviour:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {fileID: 0}
  m_PrefabInstance: {fileID: 0}
  m_PrefabAsset: {fileID: 0}
  m_GameObject: {fileID: 500000}
  m_Enabled: 1
  m_EditorHideFlags: 0
  m_Script: {fileID: 11500000, guid: f4688fdb7df04437aeb4b886b6f2c0a0, type: 3}
  m_Name:
  m_EditorClassIdentifier:
  m_Material: {fileID: 0}
  m_Color: {r: 1, g: 1, b: 1, a: 1}
  m_RaycastTarget: 1
  m_RaycastPadding: {x: 0, y: 0, z: 0, w: 0}
  m_Maskable: 1
  m_OnCullStateChanged:
    m_PersistentCalls:
      m_Calls: []
  m_text: 'Humanoid Robot

    State: IDLE'
  m_isRightToLeft: 0
  m_fontAsset: {fileID: 11400000, guid: 8f586378b4e144a9851e7b34d9b748ee, type: 2}
  m_sharedMaterials:
  - {fileID: 2180264, guid: 8f586378b4e144a9851e7b34d9b748ee, type: 2}
  m_fontSharedMaterials: []
  m_fontMaterial: {fileID: 0}
  m_fontMaterials: []
  m_fontColor32:
    serializedVersion: 2
    rgba: 4294967295
  m_fontColor: {r: 1, g: 1, b: 1, a: 1}
  m_enableVertexGradient: 0
  m_colorMode: 3
  m_fontColorGradient:
    topLeft: {r: 1, g: 1, b: 1, a: 1}
    topRight: {r: 1, g: 1, b: 1, a: 1}
    bottomLeft: {r: 1, g: 1, b: 1, a: 1}
    bottomRight: {r: 1, g: 1, b: 1, a: 1}
  m_fontColorGradientPreset: {fileID: 0}
  m_spriteAsset: {fileID: 0}
  m_tintAllSprites: 0
  m_StyleSheet: {fileID: 0}
  m_TextStyleHashCode: -1183493901
  m_overrideHtmlColors: 0
  m_faceColor:
    serializedVersion: 2
    rgba: 4294967295
  m_fontSize: 36
  m_fontSizeBase: 36
  m_fontWeight: 400
  m_enableAutoSizing: 0
  m_fontSizeMin: 18
  m_fontSizeMax: 72
  m_fontStyle: 0
  m_HorizontalAlignment: 1
  m_VerticalAlignment: 256
  m_textAlignment: 65535
  m_characterSpacing: 0
  m_wordSpacing: 0
  m_lineSpacing: 0
  m_lineSpacingMax: 0
  m_paragraphSpacing: 0
  m_charWidthMaxAdj: 0
  m_enableWordWrapping: 1
  m_wordWrappingRatios: 0.4
  m_overflowMode: 0
  m_linkedTextComponent: {fileID: 0}
  parentLinkedComponent: {fileID: 0}
  m_enableKerning: 1
  m_enableExtraPadding: 0
  checkPaddingRequired: 0
  m_isRichText: 1
  m_parseCtrlCharacters: 1
  m_isOrthographic: 0
  m_isCullingEnabled: 0
  m_horizontalMapping: 0
  m_verticalMapping: 0
  m_uvLineOffset: 0
  m_geometrySortingOrder: 0
  m_IsTextObjectScaleStatic: 0
  m_VertexBufferAutoSizeReduction: 0
  m_useMaxVisibleDescender: 1
  m_pageToDisplay: 1
  m_margin: {x: 0, y: 0, z: 0, w: 0}
  m_isUsingLegacyAnimationComponent: 0
  m_isVolumetricText: 0
  m_hasFontAssetChanged: 0
  m_renderer: {fileID: 500003}
  m_maskType: 0
--- !u!33 &500002
MeshFilter:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {fileID: 0}
  m_PrefabInstance: {fileID: 0}
  m_PrefabAsset: {fileID: 0}
  m_GameObject: {fileID: 500000}
  m_Mesh: {fileID: 10207, guid: 0000000000000000e000000000000000, type: 0}
--- !u!23 &500003
MeshRenderer:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {fileID: 0}
  m_PrefabInstance: {fileID: 0}
  m_PrefabAsset: {fileID: 0}
  m_GameObject: {fileID: 500000}
  m_Enabled: 1
  m_CastShadows: 1
  m_ReceiveShadows: 1
  m_DynamicOccludee: 1
  m_StaticShadowCaster: 0
  m_MotionVectors: 1
  m_LightProbeUsage: 1
  m_ReflectionProbeUsage: 1
  m_RayTracingMode: 2
  m_RayTraceProcedural: 0
  m_RenderingLayerMask: 1
  m_RendererPriority: 0
  m_Materials:
  - {fileID: 10303, guid: 0000000000000000f000000000000000, type: 0}
  m_StaticBatchInfo:
    firstSubMesh: 0
    subMeshCount: 0
  m_StaticBatchRoot: {fileID: 0}
  m_ProbeAnchor: {fileID: 0}
  m_LightProbeVolumeOverride: {fileID: 0}
  m_ScaleInLightmap: 1
  m_ReceiveGI: 1
  m_PreserveUVs: 0
  m_IgnoreNormalsForChartDetection: 0
  m_ImportantGI: 0
  m_StitchLightmapSeams: 1
  m_SelectedEditorRenderState: 3
  m_MinimumChartSize: 4
  m_AutoUVMaxDistance: 0.5
  m_AutoUVMaxAngle: 89
  m_LightmapParameters: {fileID: 0}
  m_SortingLayerID: 0
  m_SortingLayer: 0
  m_SortingOrder: 0
  m_AdditionalVertexStreams: {fileID: 0}
--- !u!4 &500004
Transform:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {fileID: 0}
  m_PrefabInstance: {fileID: 0}
  m_PrefabAsset: {fileID: 0}
  m_GameObject: {fileID: 500000}
  m_LocalRotation: {x: 0, y: 0, z: 0, w: 1}
  m_LocalPosition: {x: 0, y: 0, z: 0}
  m_LocalScale: {x: 1, y: 1, z: 1}
  m_ConstrainProportionsScale: 0
  m_Children: []
  m_Father: {fileID: 0}
  m_RootOrder: 2
  m_LocalEulerAnglesHint: {x: 0, y: 0, z: 0}
```

### 4. Integration Script (Python/ROS)

Create a Python script that demonstrates how to integrate all components:

```python
#!/usr/bin/env python

"""
Integrated Digital Twin Example
This script demonstrates the integration of Gazebo physics, Unity rendering,
and sensor simulation in a unified system.
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import cv2
from cv_bridge import CvBridge
import threading
import time

class IntegratedDigitalTwin:
    def __init__(self):
        rospy.init_node('integrated_digital_twin', anonymous=True)

        # Initialize bridge for image conversion
        self.bridge = CvBridge()

        # Robot state variables
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.robot_velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, omega
        self.lidar_data = None
        self.camera_data = None
        self.imu_data = None
        self.fused_state = None

        # Publishers for Unity visualization
        self.unity_pose_pub = rospy.Publisher('/unity/robot_pose', PoseStamped, queue_size=1)
        self.unity_status_pub = rospy.Publisher('/unity/status', String, queue_size=1)

        # Publishers for sensor fusion
        self.fused_pose_pub = rospy.Publisher('/fused_pose', PoseStamped, queue_size=1)

        # Subscribers for sensor data
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher for robot control
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initialize Kalman filter for sensor fusion
        self.initialize_kalman_filter()

        print("Integrated Digital Twin initialized successfully")

    def initialize_kalman_filter(self):
        """Initialize Kalman filter for sensor fusion"""
        # State vector: [x, y, theta, vx, vy, omega]
        self.state_dim = 6
        self.obs_dim = 6  # Position and orientation from different sources

        # Initial state and covariance
        self.x = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim) * 1000.0

        # Process and measurement noise
        self.Q = np.eye(self.state_dim) * 0.1  # Process noise
        self.R = np.eye(self.obs_dim) * 1.0    # Measurement noise

        # Measurement matrix (direct observation of position/orientation)
        self.H = np.eye(self.obs_dim, self.state_dim)

        print("Kalman filter initialized for sensor fusion")

    def lidar_callback(self, msg):
        """Process LiDAR data"""
        self.lidar_data = msg
        # Extract key features from LiDAR data
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]

        if len(valid_ranges) > 0:
            # Calculate distance to nearest obstacle
            min_distance = np.min(valid_ranges)
            # Publish status to Unity
            if min_distance < 1.0:
                self.unity_status_pub.publish("OBSTACLE_DETECTED: {:.2f}m".format(min_distance))
            else:
                self.unity_status_pub.publish("CLEAR_PATH: {:.2f}m".format(min_distance))

    def camera_callback(self, msg):
        """Process camera data"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process image for navigation (simplified example)
            height, width = cv_image.shape[:2]
            center_region = cv_image[int(height*0.4):int(height*0.6), int(width*0.3):int(width*0.7)]

            # Simple color-based object detection (red objects)
            hsv = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 50, 50])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)

            lower_red = np.array([170, 50, 50])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red, upper_red)

            mask = mask1 + mask2
            red_pixels = cv2.countNonZero(mask)

            if red_pixels > 100:  # If significant red detected
                self.unity_status_pub.publish("TARGET_DETECTED: {} pixels".format(red_pixels))
            else:
                self.unity_status_pub.publish("NO_TARGET_DETECTED")

            self.camera_data = cv_image

        except Exception as e:
            rospy.logerr("Error processing camera data: %s", str(e))

    def imu_callback(self, msg):
        """Process IMU data"""
        self.imu_data = msg

        # Extract orientation from IMU
        orientation_q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        _, _, theta = euler_from_quaternion(orientation_q)

        # Extract angular velocity
        omega = msg.angular_velocity.z

        # Update robot state with IMU data
        if self.fused_state is not None:
            self.fused_state[2] = theta  # Update orientation
            self.fused_state[5] = omega  # Update angular velocity

    def odom_callback(self, msg):
        """Process odometry data"""
        # Extract position and orientation from odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        orientation_q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, theta = euler_from_quaternion(orientation_q)

        # Extract velocities
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z

        # Update robot state
        self.robot_pose = np.array([x, y, theta])
        self.robot_velocity = np.array([vx, vy, omega])

        # Publish to Unity visualization
        unity_pose = PoseStamped()
        unity_pose.header.stamp = rospy.Time.now()
        unity_pose.header.frame_id = "map"
        unity_pose.pose.position.x = x
        unity_pose.pose.position.y = y
        unity_pose.pose.position.z = 0.1  # Slightly above ground
        unity_pose.pose.orientation.w = 1.0  # Simplified orientation
        self.unity_pose_pub.publish(unity_pose)

    def predict_step(self, dt):
        """Kalman filter prediction step"""
        if dt <= 0:
            return

        # State transition model (constant velocity model)
        F = np.eye(self.state_dim)
        F[0, 3] = dt  # x = x + vx*dt
        F[1, 4] = dt  # y = y + vy*dt
        F[2, 5] = dt  # theta = theta + omega*dt

        # Predict state
        self.x = F.dot(self.x)

        # Predict covariance
        self.P = F.dot(self.P).dot(F.T) + self.Q

    def update_step(self, measurement):
        """Kalman filter update step"""
        if measurement is None or len(measurement) != self.obs_dim:
            return

        # Innovation
        y = measurement - self.H.dot(self.x)

        # Innovation covariance
        S = self.H.dot(self.P).dot(self.H.T) + self.R

        # Kalman gain
        K = self.P.dot(self.H.T).dot(np.linalg.inv(S))

        # Update state
        self.x = self.x + K.dot(y)

        # Update covariance
        I = np.eye(len(self.x))
        self.P = (I - K.dot(self.H)).dot(self.P)

        # Store fused state
        self.fused_state = self.x.copy()

    def sensor_fusion_loop(self):
        """Main sensor fusion loop"""
        rate = rospy.Rate(50)  # 50 Hz
        last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            last_time = current_time

            if dt > 0:
                # Prediction step
                self.predict_step(dt)

                # Create measurement vector from available sensors
                measurement = np.zeros(self.obs_dim)
                measurement_valid = False

                # Use odometry as primary position source
                if hasattr(self, 'robot_pose') and self.robot_pose is not None:
                    measurement[0:3] = [self.robot_pose[0], self.robot_pose[1], self.robot_pose[2]]
                    measurement_valid = True

                # Use IMU as primary orientation and angular velocity source
                if self.imu_data is not None:
                    orientation_q = [self.imu_data.orientation.x, self.imu_data.orientation.y,
                                   self.imu_data.orientation.z, self.imu_data.orientation.w]
                    _, _, theta = euler_from_quaternion(orientation_q)
                    omega = self.imu_data.angular_velocity.z
                    measurement[2] = theta
                    measurement[5] = omega
                    measurement_valid = True

                # Update if we have valid measurements
                if measurement_valid:
                    self.update_step(measurement)

                    # Publish fused state
                    if self.fused_state is not None:
                        fused_pose = PoseStamped()
                        fused_pose.header.stamp = rospy.Time.now()
                        fused_pose.header.frame_id = "map"
                        fused_pose.pose.position.x = self.fused_state[0]
                        fused_pose.pose.position.y = self.fused_state[1]
                        fused_pose.pose.position.z = 0.1

                        # Convert orientation to quaternion
                        quat = quaternion_from_euler(0, 0, self.fused_state[2])
                        fused_pose.pose.orientation.x = quat[0]
                        fused_pose.pose.orientation.y = quat[1]
                        fused_pose.pose.orientation.z = quat[2]
                        fused_pose.pose.orientation.w = quat[3]

                        self.fused_pose_pub.publish(fused_pose)

            rate.sleep()

    def navigation_controller(self):
        """Simple navigation controller"""
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            cmd_vel = Twist()

            # Simple navigation behavior based on sensor inputs
            if self.lidar_data is not None:
                # Get ranges in front of robot (simplified)
                front_ranges = self.lidar_data.ranges[330:390]  # 60 degree front sector
                front_ranges = [r for r in front_ranges if self.lidar_data.range_min <= r <= self.lidar_data.range_max]

                if len(front_ranges) > 0:
                    min_front_dist = min(front_ranges)

                    if min_front_dist < 1.0:  # Obstacle too close
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.5  # Turn right
                    else:
                        cmd_vel.linear.x = 0.5  # Move forward
                        cmd_vel.angular.z = 0.0

            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

    def run(self):
        """Run the integrated digital twin system"""
        print("Starting integrated digital twin system...")

        # Start sensor fusion in a separate thread
        fusion_thread = threading.Thread(target=self.sensor_fusion_loop)
        fusion_thread.daemon = True
        fusion_thread.start()

        # Start navigation controller in a separate thread
        nav_thread = threading.Thread(target=self.navigation_controller)
        nav_thread.daemon = True
        nav_thread.start()

        print("Digital twin system running. Press Ctrl+C to stop.")

        try:
            # Keep the main thread alive
            while not rospy.is_shutdown():
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nShutting down integrated digital twin system...")

def main():
    """Main function to run the integrated example"""
    twin = IntegratedDigitalTwin()
    twin.run()

if __name__ == '__main__':
    main()
```

### 5. Launch File

Create a ROS launch file to start the complete system:

```xml
<launch>
  <!-- Start Gazebo with the integrated world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find your_robot_description)/worlds/integrated_demo_world.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- Spawn the integrated humanoid robot -->
  <param name="robot_description" command="$(find xacro)/xacro '$(find your_robot_description)/urdf/integrated_humanoid.urdf.xacro'" />

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
        args="-param robot_description -urdf -model integrated_humanoid -x 0 -y 0 -z 0.1" />

  <!-- Start the integrated digital twin node -->
  <node name="integrated_digital_twin" pkg="your_robot_control" type="integrated_digital_twin.py" output="screen" />

  <!-- Start RViz for visualization -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find your_robot_description)/rviz/integrated_demo.rviz" />

  <!-- Start the Unity ROS bridge (if available) -->
  <!-- This would connect to a Unity application running separately -->
  <node name="ros_tcp_endpoint" pkg="ros_tcp_endpoint" type="default_server_endpoint.py" args="0.0.0.0 10000" />
</launch>
```

## Integration Validation

### System Validation Steps

1. **Physics-Rendering Synchronization**:
   - Verify that robot movements in Gazebo are accurately reflected in Unity
   - Check that coordinate systems align properly between systems
   - Validate timing synchronization between physics and rendering

2. **Sensor Data Consistency**:
   - Confirm that LiDAR data corresponds to the actual environment
   - Verify that camera images show the correct scene
   - Validate IMU readings match physical motion

3. **Real-time Performance**:
   - Monitor system performance under various loads
   - Verify that all components maintain real-time operation
   - Check for any synchronization issues

4. **Integration Quality**:
   - Test navigation and manipulation tasks
   - Validate sensor fusion accuracy
   - Confirm that the digital twin behaves as expected

## Advanced Integration Features

### Dynamic Environment Updates
The system supports real-time environment changes that are reflected in both physics and rendering systems simultaneously.

### Multi-Robot Support
The architecture scales to support multiple robots with shared environments and coordinated tasks.

### Hardware-in-the-Loop
The system can be extended to incorporate real hardware components for mixed real-simulation scenarios.

## Troubleshooting Common Issues

### Synchronization Problems
- Check ROS network configuration
- Verify timing parameters in both systems
- Monitor system performance for bottlenecks

### Sensor Data Discrepancies
- Validate coordinate frame transformations
- Check sensor mounting configurations
- Verify noise model parameters

### Performance Issues
- Optimize update rates for each component
- Consider reducing simulation fidelity if needed
- Monitor CPU and memory usage

## Best Practices for Integration

1. **Modular Design**: Keep components loosely coupled for easier maintenance
2. **Standard Interfaces**: Use ROS standards for communication
3. **Error Handling**: Implement robust error handling and recovery
4. **Performance Monitoring**: Continuously monitor system performance
5. **Validation**: Regularly validate simulation against real-world data

## Conclusion

This integrated example demonstrates a complete digital twin system that combines physics simulation, high-fidelity rendering, and comprehensive sensor simulation. The system provides a realistic environment for developing and testing robotic algorithms before deployment on real hardware.

The modular architecture allows for easy extension and modification while maintaining the integration between all components. This approach enables comprehensive testing of robotic systems in a safe, repeatable, and cost-effective environment.

---

**Next**: [Validation Report](./validation-report.md) or [Cross-References](./cross-references.md)