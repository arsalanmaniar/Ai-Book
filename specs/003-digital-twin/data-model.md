# Data Model: Module 2 "The Digital Twin (Gazebo & Unity)"

## Overview
This document defines the conceptual data models relevant to Module 2 "The Digital Twin (Gazebo & Unity)". Since this is an educational module, the "data model" refers to the key concepts and structures that students need to understand about simulation.

## Core Entities

### Physics Simulation Model
- **Definition**: Representation of physical properties and behaviors in simulation
- **Key Properties**:
  - Mass: Physical weight of objects (kg)
  - Inertia: Resistance to rotational motion
  - Friction: Coefficient of surface interaction
  - Restitution: Bounciness/elasticity (0-1 range)
- **Relationships**: Applied to objects in the simulation environment to determine physical behavior

### Collision Model
- **Definition**: Geometric representation used for collision detection
- **Key Properties**:
  - Geometry Type: Box, sphere, cylinder, mesh, etc.
  - Dimensions: Size parameters for the collision shape
  - Surface Properties: Friction coefficients, contact parameters
- **Relationships**: Associated with visual models to provide physical interaction

### Gazebo World
- **Definition**: Environment definition containing models, lighting, and physics properties
- **Key Properties**:
  - Gravity: 3D vector defining gravitational force
  - Models: Collection of robots and objects in the world
  - Lighting: Ambient and directional light sources
  - Physics Engine: Configuration of the underlying physics simulation
- **Relationships**: Contains multiple models and defines their interactions

### Unity Scene
- **Definition**: 3D environment in Unity containing objects, lighting, and camera settings
- **Key Properties**:
  - GameObjects: Collection of 3D objects in the scene
  - Lighting: Directional, point, and ambient lights
  - Camera: Viewpoint and rendering settings
  - Materials: Surface properties and textures
- **Relationships**: Contains visual representations of simulated elements

### Sensor Model
- **Definition**: Simulation of real-world sensors with realistic characteristics
- **Key Properties**:
  - Sensor Type: LiDAR, depth camera, IMU, etc.
  - Noise Parameters: Characteristics to simulate real sensor noise
  - Range: Detection limits (min/max distance)
  - Resolution: Precision of measurements
- **Relationships**: Attached to robot models to provide perception capabilities

### LiDAR Simulation
- **Definition**: Simulated Light Detection and Ranging sensor
- **Key Properties**:
  - Range: Maximum detection distance
  - Resolution: Angular resolution of measurements
  - Scan Pattern: Horizontal and vertical field of view
  - Noise Model: Parameters to simulate real-world measurement errors
- **Relationships**: Produces point cloud data for environment perception

### Depth Camera Simulation
- **Definition**: Simulated 3D camera that captures depth information
- **Key Properties**:
  - Resolution: Image width and height in pixels
  - Field of View: Angular coverage of the camera
  - Depth Range: Min/max distance for depth measurements
  - Noise Characteristics: Realistic depth measurement errors
- **Relationships**: Produces RGB-D data (color + depth) for scene understanding

### IMU Simulation
- **Definition**: Simulated Inertial Measurement Unit for orientation and acceleration
- **Key Properties**:
  - Accelerometer: Linear acceleration measurements
  - Gyroscope: Angular velocity measurements
  - Magnetometer: Magnetic field measurements (optional)
  - Noise Parameters: Realistic sensor noise and drift characteristics
- **Relationships**: Provides robot state information for navigation and control

## State Transitions

### Simulation States
1. **Configuration** → **Initialization**: Loading world/model definitions
2. **Initialization** → **Running**: Starting the physics simulation
3. **Running** → **Paused**: Temporarily stopping simulation
4. **Running/Paused** → **Stopped**: Ending the simulation session

### Sensor States
1. **Inactive** → **Calibrating**: Initial sensor setup and calibration
2. **Calibrating** → **Active**: Sensor ready to produce data
3. **Active** → **Error**: Sensor malfunction or data quality issue
4. **Error** → **Active**: Recovery and resumption of normal operation

## Validation Rules

### For Physics Simulation
- Mass values must be positive
- Collision models must be closed geometry (no gaps)
- Joint limits must be physically reasonable
- Gravity parameters must match real-world values

### For Sensor Simulation
- Sensor ranges must be within realistic limits
- Noise parameters should match real sensor specifications
- Update rates should be appropriate for sensor type
- Field of view values must be within physical limits

### For Unity Rendering
- Materials must have appropriate physical properties
- Lighting should be realistic for the scene
- Camera settings should match real-world equivalents
- Asset scales should be consistent across the scene

## Relationships

```
Gazebo World --(contains)--> Physics Simulation Model
             --(contains)--> Collision Model
             --(contains)--> Sensor Model

Unity Scene --(contains)--> Visual Representation
          --(includes)--> Lighting Model
          --(includes)--> Camera Settings

Physics Simulation Model --(interacts with)--> Collision Model
                       --(affects)--> Sensor Model

Sensor Model --(produces)--> Sensor Data Stream
           --(attached to)--> Robot Model

LiDAR Simulation --(generates)--> Point Cloud Data
Depth Camera --(generates)--> RGB-D Data
IMU Simulation --(generates)--> Orientation/Acceleration Data
```