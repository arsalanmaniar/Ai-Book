---
title: Gravity and Collision Modeling
sidebar_label: Gravity and Collision Modeling
---

# Gravity and Collision Modeling

**Module**: Module 2 "The Digital Twin (Gazebo & Unity)"
**Chapter**: 4 - Physics Simulation in Gazebo
**Section**: Gravity and Collision Modeling
**Created**: 2025-12-20
**Last Updated**: 2025-12-20

## Overview

Gravity and collision modeling are fundamental aspects of physics simulation that determine how objects move and interact in the simulated environment. This section covers the principles and implementation of realistic gravity and collision behavior in Gazebo, with special attention to humanoid robot applications.

## Gravity Modeling

### Understanding Gravity in Simulation

Gravity is a fundamental force that affects all objects with mass. In simulation:

- Gravity provides a constant downward acceleration (9.81 m/s² on Earth)
- It affects robot balance, locomotion, and manipulation
- Proper gravity modeling is essential for realistic humanoid behavior

### Configuring Gravity in Gazebo

Gravity is defined in the world file and applies to all objects in the simulation:

```xml
<!-- Example gravity configuration in SDF world file -->
<world name="physics_world">
  <physics type="ode">
    <gravity>0 0 -9.8</gravity>
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
</world>
```

### Gravity Vector Components

The gravity vector is defined as (x, y, z) in meters per second squared:

- **X**: Gravity component in the X direction (typically 0)
- **Y**: Gravity component in the Y direction (typically 0)
- **Z**: Gravity component in the Z direction (typically -9.8 for Earth)

### Custom Gravity Scenarios

For educational purposes, you can simulate different gravity environments:

```xml
<!-- Moon gravity (1/6 of Earth) -->
<gravity>0 0 -1.63</gravity>

<!-- Mars gravity -->
<gravity>0 0 -3.71</gravity>

<!-- Zero gravity (space simulation) -->
<gravity>0 0 0</gravity>

<!-- Custom direction (for special experiments) -->
<gravity>-0.5 0 -9.3</gravity>
```

### Gravity and Robot Balance

For humanoid robots, gravity affects:

- **Center of Mass (CoM)**: Critical for balance and stability
- **Zero Moment Point (ZMP)**: Determines stable walking patterns
- **Joint torques**: Required to maintain posture against gravity
- **Foot placement**: For stable standing and walking

## Collision Detection

### Collision Geometry Types

Gazebo supports various collision geometries:

#### Primitive Shapes
- **Box**: Rectangular parallelepiped
- **Sphere**: Perfect spherical shape
- **Cylinder**: Cylindrical shape
- **Capsule**: Cylinder with hemispherical ends

#### Complex Shapes
- **Mesh**: Arbitrary 3D mesh for complex geometries
- **Plane**: Infinite flat surface (typically for ground)
- **Heightmap**: Terrain represented as height values

```xml
<!-- Example collision geometries -->
<collision name="box_collision">
  <geometry>
    <box><size>0.1 0.2 0.3</size></box>
  </geometry>
</collision>

<collision name="sphere_collision">
  <geometry>
    <sphere><radius>0.05</radius></sphere>
  </geometry>
</collision>

<collision name="mesh_collision">
  <geometry>
    <mesh><uri>model://robot/meshes/complex_shape.stl</uri></mesh>
  </geometry>
</collision>
```

### Collision Detection Algorithms

Gazebo uses several collision detection approaches:

#### Broad Phase
- Quickly eliminates non-colliding pairs
- Uses spatial partitioning (bounding volume hierarchies)
- Reduces computational complexity

#### Narrow Phase
- Precise collision detection for potential pairs
- Computes contact points and penetration depth
- Determines collision response parameters

### Contact Point Generation

When collisions occur, Gazebo generates contact points:

- **Contact position**: Location where objects touch
- **Contact normal**: Direction perpendicular to contact surface
- **Penetration depth**: How much objects overlap
- **Contact force**: Force required to separate objects

## Collision Parameters

### Material Properties

Collision behavior depends on material properties:

#### Friction Coefficients
- **Static friction (mu)**: Resistance to initial sliding
- **Dynamic friction (mu2)**: Resistance to continued sliding
- **Torsional friction**: Resistance to twisting motion

```xml
<gazebo reference="link_name">
  <collision>
    <surface>
      <friction>
        <ode>
          <mu>0.5</mu>
          <mu2>0.4</mu2>
        </ode>
      </friction>
    </surface>
  </collision>
</gazebo>
```

#### Restitution (Bounciness)
- **Restitution coefficient**: How much energy is preserved during collision
- Range: 0 (perfectly inelastic) to 1 (perfectly elastic)
- Affects bounce behavior and energy conservation

```xml
<gazebo reference="link_name">
  <collision>
    <surface>
      <bounce>
        <restitution_coefficient>0.2</restitution_coefficient>
        <threshold>1.0</threshold>
      </bounce>
    </surface>
  </collision>
</gazebo>
```

### Contact Parameters

Fine-tune contact behavior with these parameters:

#### Contact Stiffness and Damping
- **Stiffness**: How "hard" the contact is (spring constant)
- **Damping**: Energy dissipation during contact
- **Desired penetration**: Target overlap during steady contact

```xml
<gazebo reference="link_name">
  <collision>
    <surface>
      <contact>
        <ode>
          <soft_cfm>0.001</soft_cfm>
          <soft_erp>0.2</soft_erp>
          <kp>1e+6</kp>  <!-- Contact stiffness -->
          <kd>100</kd>   <!-- Damping coefficient -->
        </ode>
      </contact>
    </surface>
  </collision>
</gazebo>
```

## Collision Performance

### Simplified vs. Detailed Collision Models

For performance reasons, collision models can differ from visual models:

#### Visual vs. Collision Geometry
- **Visual**: High-resolution meshes for rendering
- **Collision**: Simplified shapes for physics calculations
- Same link can have different visual and collision representations

```xml
<link name="complex_link">
  <!-- High-resolution visual model -->
  <visual>
    <geometry>
      <mesh><uri>model://robot/meshes/detailed_model.stl</uri></mesh>
    </geometry>
  </visual>

  <!-- Simplified collision model -->
  <collision>
    <geometry>
      <cylinder><radius>0.05</radius><length>0.2</length></cylinder>
    </geometry>
  </collision>
</link>
```

### Hierarchical Collision Detection

For complex objects, use multiple collision elements:

```xml
<link name="arm_link">
  <!-- Multiple collision elements for complex shape -->
  <collision name="upper_arm_collision">
    <geometry><cylinder><radius>0.04</radius><length>0.15</length></cylinder></geometry>
  </collision>
  <collision name="lower_arm_collision">
    <geometry><cylinder><radius>0.03</radius><length>0.12</length></cylinder></geometry>
  </collision>
</link>
```

## Humanoid-Specific Collision Considerations

### Ground Contact Modeling

For humanoid robots, ground contact is critical:

#### Foot Contact
- Model foot-ground interaction for stable walking
- Consider sole geometry and material properties
- Account for different contact scenarios (toe, heel, full foot)

```xml
<link name="foot">
  <collision name="foot_contact">
    <geometry>
      <box><size>0.15 0.08 0.02</size></box>
    </geometry>
    <surface>
      <friction>
        <ode><mu>0.8</mu><mu2>0.7</mu2></ode>
      </friction>
    </surface>
  </collision>
</link>
```

#### Balance and Stability
- Proper collision modeling affects balance control
- Friction coefficients impact walking stability
- Contact points influence Center of Pressure (CoP)

### Joint Limit Collisions

Model joint limits as collision constraints:

- Prevent joint over-extension
- Simulate physical joint stops
- Provide realistic joint behavior

### Whole-Body Collision Avoidance

For humanoid robots with many degrees of freedom:

- Model self-collision detection
- Implement collision avoidance algorithms
- Consider computational complexity

## Collision Detection Optimization

### Performance Considerations

Balance accuracy with performance:

#### Collision Mesh Simplification
- Use convex hulls for complex shapes
- Reduce polygon count for collision meshes
- Use primitive shapes where possible

#### Spatial Partitioning
- Gazebo automatically partitions space
- Organize models to minimize unnecessary collision checks
- Consider collision groups for complex scenes

### Accuracy vs. Performance Trade-offs

| Setting | Performance | Accuracy | Use Case |
|---------|-------------|----------|----------|
| High accuracy | Slow | High | Precise manipulation tasks |
| Medium accuracy | Balanced | Good | General robotics |
| Low accuracy | Fast | Lower | Real-time control |

## Debugging Collision Issues

### Common Collision Problems

#### Objects Falling Through Surfaces
- Check collision geometries are properly defined
- Verify coordinate system alignment
- Ensure collision and visual geometries are positioned correctly

#### Objects Phasing Through Each Other
- Reduce time step size
- Increase contact stiffness
- Add more collision elements for complex shapes

#### Unstable Oscillations
- Adjust solver parameters
- Increase damping coefficients
- Verify mass and inertia values

#### Excessive Penetration
- Increase contact stiffness (kp)
- Reduce time step
- Check for mass ratio issues

### Debugging Tools

#### Gazebo Visualization
- Enable contact visualization to see contact points
- Use wireframe mode to verify collision geometries
- Monitor joint forces and torques

#### Logging and Analysis
- Log contact forces for analysis
- Monitor energy conservation
- Track penetration depths

## Validation and Testing

### Gravity Validation
1. **Free Fall Test**: Verify objects accelerate at 9.8 m/s²
2. **Pendulum Test**: Confirm pendulum period matches theory
3. **Projectile Motion**: Validate trajectory calculations

### Collision Validation
1. **Restitution Test**: Verify bounce heights match coefficients
2. **Friction Test**: Confirm sliding behavior matches coefficients
3. **Stability Test**: Ensure objects remain stable when they should

## Best Practices

### For Humanoid Robots
1. **Start Simple**: Begin with basic shapes, add complexity gradually
2. **Realistic Materials**: Use friction coefficients matching real materials
3. **Proper Mass Distribution**: Ensure realistic CoM for humanoid balance
4. **Validate Behavior**: Test with known physical scenarios

### Performance Optimization
1. **Simplified Collision**: Use simple shapes for collision detection
2. **Appropriate Time Step**: Balance accuracy with performance
3. **Solver Tuning**: Adjust parameters for specific simulation needs
4. **Monitor Resources**: Track CPU and memory usage

## Exercises

1. **Gravity Experiment**: Create objects with different masses and verify they fall at the same rate
2. **Friction Investigation**: Test different friction coefficients and observe sliding behavior
3. **Stability Challenge**: Create a humanoid model and test its stability under different gravity conditions
4. **Collision Detection**: Build a complex scene and optimize collision detection for performance

## Summary

Gravity and collision modeling form the backbone of realistic physics simulation. Proper configuration of these elements is essential for humanoid robot simulation, affecting balance, locomotion, and interaction with the environment. Understanding the trade-offs between accuracy and performance helps create effective simulation environments.

---

**Next**: [Humanoid Simulation](./humanoid-simulation.md)
