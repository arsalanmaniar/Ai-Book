---
title: Physics Fundamentals in Simulation
sidebar_label: Physics Fundamentals
---

# Physics Fundamentals in Simulation

**Module**: Module 2 "The Digital Twin (Gazebo & Unity)"
**Chapter**: 4 - Physics Simulation in Gazebo
**Section**: Physics Fundamentals
**Created**: 2025-12-20
**Last Updated**: 2025-12-20

## Overview

This section introduces the fundamental physics concepts that underpin simulation in Gazebo. Understanding these concepts is crucial for creating realistic and stable simulations for humanoid robots. We'll cover the core principles that govern how objects behave in simulated environments.

## Core Physics Concepts

### 1. Mass and Inertia

**Mass** represents the amount of matter in an object and determines how it responds to forces. In robotics simulation:

- Mass affects how robots move when forces are applied
- It influences collision behavior and stability
- For humanoid robots, mass distribution is critical for balance

**Inertia** describes an object's resistance to changes in rotational motion. In simulation:

- Inertia tensors define how objects rotate when torques are applied
- Proper inertia values are essential for realistic joint behavior
- For humanoid robots, inertia affects walking stability and manipulation

```xml
<!-- Example mass and inertia definition in URDF -->
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0.0" ixz="0.0"
           iyy="0.01" iyz="0.0" izz="0.01"/>
</inertial>
```

### 2. Forces and Torques

**Forces** cause linear acceleration according to Newton's second law (F = ma). In simulation:

- Gravity applies a constant downward force
- Actuator forces move robot joints
- Contact forces arise from collisions

**Torques** cause angular acceleration. In robotics:

- Joint actuators apply torques to move limbs
- Gravity torques affect balance
- External torques from contact with environment

### 3. Kinematics vs. Dynamics

**Kinematics** describes motion without considering forces:
- Forward kinematics: joint angles → end-effector position
- Inverse kinematics: end-effector position → required joint angles
- Purely geometric relationships

**Dynamics** includes forces and their effects:
- How forces cause motion
- How mass and inertia affect movement
- How collisions and contacts influence behavior

## Physics Engines in Gazebo

Gazebo supports multiple physics engines, each with different strengths:

### Open Dynamics Engine (ODE)
- Fast and stable for most applications
- Good for rigid body simulation
- Default choice for many robotics applications
- Handles joint constraints well

### Bullet Physics
- More robust collision detection
- Better for complex contact scenarios
- Handles soft body simulation better
- Good for humanoid robot applications

### DART (Dynamic Animation and Robotics Toolkit)
- Advanced contact modeling
- Stable for complex articulated systems
- Good for bipedal locomotion
- More modern and actively developed

## Simulation Parameters

### Time Step Configuration

The physics simulation advances in discrete time steps. Key parameters:

- **Max Step Size**: Duration of each simulation step (typically 0.001s)
- **Real Time Factor**: How fast simulation runs relative to real time
- **Accuracy**: Trade-off between performance and precision

```xml
<!-- Example physics configuration in SDF -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

### Solver Parameters

Physics engines use numerical solvers to compute forces and motions:

- **Iterations**: Number of solver iterations per step (more = more accurate but slower)
- **Sor PGS Precon Iters**: Preconditioning iterations
- **Sor PGS Iters**: Solver iterations
- **Sor PGS Omega**: Relaxation parameter

## Rigid Body Simulation

In Gazebo, all objects are treated as rigid bodies:

### Properties of Rigid Bodies
- Shape and geometry remain constant
- Mass and inertia are fixed
- Internal forces maintain structural integrity
- Only external forces affect motion

### Collision and Visual Separation
- **Collision geometry**: Used for physics calculations
- **Visual geometry**: Used for rendering
- Both can be defined independently in URDF/SDF

```xml
<!-- Example collision and visual separation -->
<link name="link_name">
  <visual>
    <geometry>
      <mesh filename="complex_shape.dae"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.1 0.1 0.1"/> <!-- Simplified collision -->
    </geometry>
  </collision>
</link>
```

## Joint Modeling

Joints connect rigid bodies and constrain their relative motion:

### Joint Types
- **Revolute**: Rotational motion around single axis
- **Prismatic**: Linear motion along single axis
- **Fixed**: No relative motion (welds bodies together)
- **Continuous**: Unlimited rotational motion
- **Floating**: 6 DOF unconstrained motion
- **Planar**: Motion constrained to a plane

### Joint Dynamics
- **Damping**: Resistance to motion (like friction)
- **Friction**: Static friction threshold
- **Spring stiffness**: Elastic force proportional to displacement
- **Spring reference**: Rest position for spring force

## Contact Modeling

When objects touch, contact forces arise:

### Contact Parameters
- **Stiffness**: How hard the contact is (spring constant)
- **Damping**: Energy dissipation during contact
- **Mu (friction coefficient)**: Resistance to sliding
- **Mu2**: Friction coefficient in the second direction
- **Slip1/Slip2**: Velocity-based slip parameters

### Contact Processing
- Collision detection identifies potential contacts
- Contact points are computed
- Contact forces are calculated and applied
- Object motion is updated based on forces

## Stability Considerations

### Time Step Selection
- Too large: Unstable simulation, objects may pass through each other
- Too small: Slow simulation, high computational cost
- Rule of thumb: At least 10 steps per oscillation period

### Mass Ratio Issues
- Large mass ratios can cause instability
- Keep mass ratios reasonable (less than 100:1)
- Use fixed joints instead of very heavy springs when possible

### Numerical Accuracy
- Increase solver iterations for better accuracy
- Monitor energy conservation in closed systems
- Validate simulation behavior against known physics

## Humanoid-Specific Considerations

### Balance and Stability
- Center of mass (CoM) position is critical
- Zero Moment Point (ZMP) for walking stability
- Proper mass distribution affects control

### Actuation Modeling
- Joint friction affects control precision
- Gear ratios affect actuator characteristics
- Motor dynamics influence response

## Quality Assurance

### Validation Techniques
1. **Energy Conservation**: Verify energy is conserved in frictionless systems
2. **Known Solutions**: Compare with analytical solutions when possible
3. **Parameter Sensitivity**: Test behavior with parameter variations
4. **Stability Testing**: Verify simulation remains stable over long runs

### Common Issues
- Objects falling through surfaces: Check collision geometries
- Unstable oscillations: Reduce time step or increase damping
- Penetration: Increase contact stiffness or reduce time step
- Drifting: Check for numerical errors in constraints

## Best Practices

1. **Start Simple**: Begin with basic models and add complexity gradually
2. **Validate Components**: Test individual joints and links before integration
3. **Use Realistic Parameters**: Base values on physical measurements
4. **Monitor Performance**: Balance accuracy with computational efficiency
5. **Document Assumptions**: Record simplifications and limitations

## Exercises

1. **Simple Pendulum**: Create a pendulum and verify its period matches theoretical predictions
2. **Mass-Spring System**: Implement a mass-spring system and verify oscillation frequency
3. **Ramp Sliding**: Simulate objects sliding down ramps and verify friction effects

## Summary

Physics fundamentals form the foundation of realistic simulation. Understanding mass, inertia, forces, and contact modeling is essential for creating stable and accurate simulations. Proper configuration of physics engines and parameters ensures reliable simulation behavior for humanoid robots.

---

**Next**: [Gravity and Collision Modeling](./gravity-collision-modeling.md)