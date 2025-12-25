---
title: Assessment Questions - Physics Simulation
sidebar_label: Assessment Questions
---

# Assessment Questions - Physics Simulation

**Module**: Module 2 "The Digital Twin (Gazebo & Unity)"
**Chapter**: 4 - Physics Simulation in Gazebo
**Section**: Assessment Questions
**Created**: 2025-12-20
**Last Updated**: 2025-12-20

## Overview

This assessment evaluates your understanding of physics simulation concepts in Gazebo, with a focus on humanoid robot applications. The questions cover fundamental physics principles, simulation implementation, and practical considerations for creating realistic humanoid simulations.

## Multiple Choice Questions

### Question 1
What is the primary purpose of the Zero Moment Point (ZMP) in humanoid robotics?
A) To determine the center of mass of the robot
B) To identify the point where net moment of ground reaction forces is zero
C) To measure the maximum walking speed
D) To calculate the required joint torques

**Answer: B** - The ZMP is the point where the net moment of ground reaction forces is zero, which is critical for walking pattern generation and balance control in humanoid robots.

### Question 2
Which physics parameter in Gazebo controls the stiffness of contact interactions?
A) `damping`
B) `restitution`
C) `kp` (spring constant)
D) `mu` (friction coefficient)

**Answer: C** - The `kp` parameter (spring constant) controls the stiffness of contact interactions, determining how "hard" the contact is.

### Question 3
What is the recommended approach for handling collision detection between complex robot models?
A) Use high-resolution meshes for both visual and collision geometries
B) Use simplified shapes for collision detection while maintaining detailed visuals
C) Disable collision detection for complex models
D) Use only primitive shapes for all robot parts

**Answer: B** - Using simplified shapes for collision detection while maintaining detailed visuals provides the best performance vs. accuracy trade-off.

### Question 4
In humanoid simulation, why is mass distribution particularly important?
A) It affects only the visual appearance of the robot
B) It determines the robot's maximum speed
C) It critically affects balance, stability, and control
D) It only matters for simulation performance

**Answer: C** - Proper mass distribution is critical for balance, stability, and control in humanoid robots, affecting the center of mass and moments of inertia.

### Question 5
What is the typical range for restitution coefficient in Gazebo?
A) 0 to 1
B) -1 to 1
C) 0 to 10
D) 0 to infinity

**Answer: A** - The restitution coefficient ranges from 0 (perfectly inelastic) to 1 (perfectly elastic) in Gazebo.

## Short Answer Questions

### Question 6
Explain the difference between kinematics and dynamics in the context of robot simulation.

**Answer**: Kinematics describes motion without considering forces (forward/inverse kinematics, geometric relationships), while dynamics includes the effects of forces, mass, and inertia on motion (how forces cause acceleration, how mass affects movement).

### Question 7
What are the main challenges in simulating humanoid robot balance compared to simpler robotic systems?

**Answer**: Humanoid robots are inherently unstable with high center of mass relative to support base, require active control for balance, have many degrees of freedom creating complex dynamics, and experience rapid changes in support polygon during walking.

### Question 8
List three factors that affect the stability of physics simulation in Gazebo.

**Answer**:
1. Time step size (smaller steps improve stability but reduce performance)
2. Solver parameters (iterations, ERP, CFM affect constraint satisfaction)
3. Mass ratios between objects (extreme ratios can cause instability)

### Question 9
Describe the role of friction coefficients (mu and mu2) in Gazebo collision models.

**Answer**: The friction coefficients determine the resistance to sliding motion. `mu` is the primary friction coefficient in the primary slip direction, while `mu2` is the friction coefficient in the secondary slip direction, allowing for anisotropic friction behavior.

### Question 10
What is the significance of the real-time factor in Gazebo simulation?

**Answer**: The real-time factor indicates how fast the simulation runs relative to real time. A factor of 1.0 means the simulation runs at real-time speed, while values above 1.0 mean it runs faster than real time, and values below 1.0 mean it runs slower than real time.

## Practical Application Questions

### Question 11
You are creating a physics simulation of a humanoid robot that keeps falling over during standing. Describe a systematic approach to diagnose and fix this issue.

**Answer**:
1. Check mass distribution - ensure center of mass is properly positioned
2. Verify joint limits and dynamics - ensure joints can provide necessary torques
3. Validate control algorithms - check balance controller implementation
4. Examine contact models - ensure feet make proper contact with ground
5. Monitor sensor feedback - verify IMU and force/torque sensor models

### Question 12
Explain how you would validate that your physics simulation parameters are realistic for a humanoid robot.

**Answer**:
1. Compare simulation behavior with known physics (e.g., free fall acceleration)
2. Test with simplified models before complex humanoid
3. Validate energy conservation in appropriate scenarios
4. Compare with real robot data if available
5. Test with known inputs and verify expected outputs

### Question 13
Describe the trade-offs involved in choosing collision geometries for a humanoid robot model.

**Answer**: Complex collision geometries provide accurate physics but reduce performance, while simple geometries improve performance but may miss important contact scenarios. The choice depends on required accuracy vs. computational constraints, with common approaches using simplified shapes for collision while maintaining detailed visuals.

### Question 14
What are the key parameters to tune for achieving stable contact between a humanoid robot's feet and the ground?

**Answer**:
1. Contact stiffness (`kp`) - higher values reduce penetration but may cause instability
2. Contact damping (`kd`) - appropriate values reduce oscillations
3. Friction coefficients (`mu`, `mu2`) - realistic values for the surface material
4. Time step - smaller steps improve contact stability
5. Solver iterations - more iterations improve constraint satisfaction

### Question 15
How would you approach debugging a humanoid robot simulation that becomes unstable after running for several minutes?

**Answer**:
1. Check for energy drift by monitoring system energy over time
2. Verify mass and inertia parameters are stable
3. Look for numerical integration errors
4. Check for constraint drift in joint models
5. Monitor memory usage for potential leaks
6. Test with simpler models to isolate the issue

## Calculation Problems

### Question 16
A humanoid robot has a total mass of 60 kg with its center of mass located 0.85 m above the ground. If the robot's feet are positioned 0.2 m apart laterally, calculate the maximum horizontal force that can be applied at the center of mass before the robot tips over, assuming a friction coefficient of 0.8.

**Answer**:
The maximum horizontal force before tipping occurs when the moment from the horizontal force equals the stabilizing moment from the robot's weight:
- Tipping moment = Force × height = F × 0.85
- Stabilizing moment = Weight × half base width = (60 × 9.81) × 0.1 = 58.86 N·m
- Maximum force before tipping: F = 58.86 / 0.85 ≈ 69.2 N
- Maximum force before sliding: F = μ × Normal force = 0.8 × (60 × 9.81) ≈ 470.9 N
- The robot will tip before sliding, so the answer is 69.2 N.

### Question 17
In a simulation with a time step of 0.001 seconds, how many simulation steps occur in one second of simulated time?

**Answer**: 1 / 0.001 = 1000 steps per second of simulated time.

### Question 18
A ball with a restitution coefficient of 0.7 is dropped from a height of 1 meter. Calculate the approximate height of the first bounce.

**Answer**: The bounce height is approximately the initial height multiplied by the square of the restitution coefficient: 1.0 × (0.7)² = 0.49 meters.

## Essay Questions

### Question 19
Discuss the importance of physics validation in humanoid robot simulation and describe at least three validation techniques.

**Answer**: Physics validation is crucial for humanoid robot simulation because the simulation serves as a proxy for real-world behavior, and inaccurate simulation can lead to control strategies that fail on real robots. Validation ensures that the simulation behaves similarly to physical reality.

Validation techniques include:
1. **Component-level validation**: Testing individual joints, links, and sensors to ensure they behave as expected
2. **Known solution validation**: Comparing simulation results with analytical solutions for simple cases (e.g., pendulum motion, projectile trajectories)
3. **Real-world comparison**: Validating simulation behavior against data from real robots or physical experiments when available

### Question 20
Explain the concept of sim-to-real transfer in robotics and discuss the challenges specific to physics simulation of humanoid robots.

**Answer**: Sim-to-real transfer refers to the process of developing and testing robotic algorithms in simulation before deploying them on real robots. The goal is to reduce development time and cost while improving safety.

Challenges specific to humanoid physics simulation include:
- **Model accuracy**: Humanoid robots have complex multi-body dynamics that are difficult to model precisely
- **Contact modeling**: Foot-ground interactions are complex and affected by surface properties, shoe materials, and robot compliance
- **Balance control**: Small modeling errors can significantly affect balance, which is critical for humanoid robots
- **Actuator dynamics**: Real actuators have complex behaviors (friction, backlash, compliance) that are difficult to model
- **Sensor noise and delay**: Real sensors have noise, delay, and limited accuracy not captured in simulation
- **Environmental factors**: Real environments have unmodeled elements that affect robot behavior

## Performance Assessment

### Question 21
You need to optimize a humanoid simulation that runs at 0.3 real-time factor (too slow). Describe your optimization strategy considering both accuracy and performance.

**Answer**: Optimization strategy:
1. **Physics parameters**: Increase time step gradually while monitoring stability
2. **Collision simplification**: Use simpler shapes for collision detection where accuracy allows
3. **Solver tuning**: Reduce solver iterations while maintaining stability
4. **Model simplification**: Reduce unnecessary complexity in visual models
5. **Contact parameters**: Optimize contact stiffness and damping for performance
6. **Parallelization**: Use multiple threads where possible
7. **Selective detail**: Maintain high detail only where needed for the specific task

### Question 22
Design a test scenario to validate the physics simulation of a humanoid robot's walking behavior.

**Answer**: A comprehensive walking validation scenario would include:
1. **Steady walking**: Measure step parameters (length, width, height) and compare with expected values
2. **Balance recovery**: Apply small disturbances and verify the robot can recover
3. **Turning**: Test the robot's ability to turn while maintaining balance
4. **Different surfaces**: Test on surfaces with different friction coefficients
5. **Energy efficiency**: Monitor power consumption and walking smoothness
6. **Stability margins**: Test with varying walking speeds and step parameters
7. **Long-term stability**: Run for extended periods to check for drift or instability

## Summary Assessment

### Question 23
As a robotics engineer, you are tasked with creating a physics simulation for a new humanoid robot design. Outline your complete workflow from model creation to simulation validation.

**Answer**: Complete workflow:
1. **Model creation**: Create accurate URDF with proper mass, inertia, and joint properties
2. **Physics parameter selection**: Choose appropriate physics engine and configure parameters
3. **Collision geometry definition**: Create efficient collision models
4. **Initial validation**: Test basic physics with simple scenarios
5. **Control integration**: Implement and test basic controllers
6. **Advanced validation**: Test complex behaviors (walking, manipulation)
7. **Parameter tuning**: Optimize for accuracy and performance
8. **Documentation**: Record parameters and validation results
9. **Ongoing validation**: Continuously validate as the model evolves

### Question 24
Compare and contrast the physics simulation requirements for wheeled robots versus humanoid robots, highlighting the specific challenges of humanoid simulation.

**Answer**:

**Wheeled Robots**:
- More stable base of support
- Fewer degrees of freedom
- Continuous ground contact
- Simpler contact modeling
- Less complex balance requirements

**Humanoid Robots**:
- Inherently unstable with changing support polygon
- Many degrees of freedom requiring complex control
- Discrete foot contact with complex transitions
- Complex contact modeling with multiple contact points
- Critical balance requirements with active control needed

**Specific humanoid challenges**:
- Balance control with high center of mass
- Complex walking patterns with dynamic balance
- Multi-contact scenarios during manipulation
- Rapid constraint changes during locomotion
- High sensitivity to parameter variations

---

**Next**: [Chapter Summary](./summary.md) or [Chapter 5: Unity Rendering and Human-Robot Interaction](../chapter-5-unity-rendering/)
