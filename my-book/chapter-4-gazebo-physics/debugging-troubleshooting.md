---
title: Debugging and Troubleshooting Physics Simulation
sidebar_label: Debugging and Troubleshooting
---

# Debugging and Troubleshooting Physics Simulation

**Module**: Module 2 "The Digital Twin (Gazebo & Unity)"
**Chapter**: 4 - Physics Simulation in Gazebo
**Section**: Debugging and Troubleshooting
**Created**: 2025-12-20
**Last Updated**: 2025-12-20

## Overview

Physics simulation in Gazebo can present various challenges, especially when simulating complex systems like humanoid robots. This section provides systematic approaches to identify, diagnose, and resolve common physics simulation issues. Understanding these debugging techniques is crucial for developing reliable and stable simulation environments.

## Common Physics Simulation Issues

### 1. Object Penetration and Collision Issues

#### Problem: Objects Falling Through Surfaces
**Symptoms**:
- Robot falls through the ground plane
- Objects pass through walls or obstacles
- Unstable contact behavior

**Root Causes**:
- Missing collision geometries
- Incorrect collision parameters
- Inadequate physics time step
- Improper mass/inertia values

**Solutions**:
```xml
<!-- Ensure collision geometry is properly defined -->
<link name="link_name">
  <collision>
    <geometry>
      <box><size>0.1 0.1 0.1</size></box>
    </geometry>
    <surface>
      <contact>
        <ode>
          <kp>1e+6</kp>  <!-- Increase stiffness -->
          <kd>100</kd>   <!-- Increase damping -->
        </ode>
      </contact>
    </surface>
  </collision>
</link>
```

#### Problem: Excessive Penetration
**Symptoms**:
- Objects visibly sinking into each other
- Unstable contact forces
- Oscillating behavior at contact points

**Solutions**:
1. Increase contact stiffness (kp) parameter
2. Reduce physics time step
3. Increase damping coefficient (kd)
4. Verify mass ratios between objects

### 2. Stability and Numerical Issues

#### Problem: Simulation Instability
**Symptoms**:
- Objects exhibiting chaotic motion
- Robot joints oscillating wildly
- Simulation "exploding" after some time

**Root Causes**:
- Time step too large
- Solver parameters inadequate
- Mass ratios too extreme
- Inertia tensors improperly defined

**Solutions**:
```xml
<!-- Adjust physics parameters for better stability -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- Reduce time step -->
  <ode>
    <solver>
      <iters>200</iters>  <!-- Increase iterations -->
      <sor>1.2</sor>      <!-- Adjust relaxation parameter -->
    </solver>
    <constraints>
      <cfm>1e-5</cfm>     <!-- Constraint force mixing -->
      <erp>0.1</erp>      <!-- Error reduction parameter -->
    </constraints>
  </ode>
</physics>
```

#### Problem: Energy Drift
**Symptoms**:
- Objects gradually gaining or losing energy
- Pendulums not maintaining consistent amplitude
- Robot behavior changing over time

**Solutions**:
1. Use energy-conserving integrators
2. Monitor energy conservation in closed systems
3. Reduce numerical integration errors
4. Validate mass and inertia parameters

### 3. Humanoid-Specific Issues

#### Problem: Balance Instability
**Symptoms**:
- Humanoid robot falling over without external forces
- Joint oscillations during standing
- Unstable walking patterns

**Root Causes**:
- Center of mass too high or improperly positioned
- Joint control parameters inadequate
- Insufficient feedback control
- Inaccurate mass distribution

**Solutions**:
1. Verify CoM position relative to support polygon
2. Implement proper balance control algorithms
3. Check joint limits and dynamics
4. Validate actuator capabilities

#### Problem: Joint Limit Violations
**Symptoms**:
- Joints exceeding physical limits
- Robot configurations that shouldn't be possible
- Joint velocities exceeding limits

**Solutions**:
```xml
<!-- Properly configure joint limits -->
<joint name="joint_name" type="revolute">
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
  <dynamics damping="1.0" friction="0.1"/>
  <safety_controller k_position="10" k_velocity="10"
                    soft_lower_limit="-1.5" soft_upper_limit="1.5"/>
</joint>
```

## Debugging Tools and Techniques

### 1. Gazebo Visualization Tools

#### Contact Visualization
Enable contact point visualization to see collision interactions:

```bash
# In Gazebo GUI, enable View -> Contacts
# Or add to world file:
<rendering>
  <contacts>true</contacts>
</rendering>
```

#### Joint Force/Torque Monitoring
Monitor forces and torques at joints:

```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <joint_name>joint_name</joint_name>
  </plugin>
</gazebo>
```

### 2. Logging and Analysis

#### Enable Physics Logging
```bash
# Launch Gazebo with physics logging
gzserver --verbose your_world.world
```

#### Monitor Simulation Performance
```bash
# Check real-time factor
gz stats

# Monitor physics updates
gz topic -e /gazebo/default/physics/contacts
```

### 3. ROS 2 Integration Debugging

#### Joint State Monitoring
```bash
# Monitor joint positions and velocities
ros2 topic echo /joint_states

# Monitor IMU data for balance
ros2 topic echo /imu/data
```

## Systematic Debugging Approach

### Step 1: Problem Identification
1. **Reproduce the issue**: Document exact conditions that cause the problem
2. **Isolate the problem**: Determine which components are involved
3. **Characterize the behavior**: Note patterns, frequencies, and triggers

### Step 2: Parameter Verification
1. **Check mass and inertia values**:
   - Verify all links have mass > 0
   - Ensure inertia values are physically plausible
   - Check products of inertia are reasonable

2. **Validate joint parameters**:
   - Confirm joint limits are appropriate
   - Verify joint dynamics (damping, friction)
   - Check joint types match intended motion

3. **Review collision properties**:
   - Ensure all collidable objects have collision geometries
   - Verify friction coefficients are realistic
   - Check restitution values are in [0,1] range

### Step 3: Physics Configuration Tuning
1. **Time step adjustment**:
   - Start with 0.001s and adjust as needed
   - Monitor for stability vs. performance trade-offs

2. **Solver parameter optimization**:
   - Increase iterations if experiencing instability
   - Adjust ERP and CFM for constraint satisfaction
   - Tune SOR parameters for convergence

3. **Contact parameter tuning**:
   - Adjust stiffness and damping for contact behavior
   - Modify friction coefficients for realistic sliding
   - Set appropriate restitution for bounce behavior

### Step 4: Validation and Testing
1. **Simple test cases**: Start with basic scenarios before complex ones
2. **Incremental complexity**: Add complexity gradually
3. **Quantitative validation**: Compare with known physical behaviors
4. **Long-term stability**: Test for extended simulation periods

## Humanoid-Specific Debugging

### Balance Control Debugging
```python
# Example Python code for monitoring balance metrics
import numpy as np

def calculate_zmp(ground_reaction_forces, cop_positions):
    """Calculate Zero Moment Point for balance analysis"""
    # Implementation based on ground reaction forces
    pass

def monitor_balance_metrics(robot_state):
    """Monitor key balance indicators"""
    com_position = robot_state['com']
    cop_position = robot_state['cop']
    zmp = robot_state['zmp']

    # Check if CoM is within support polygon
    if not is_stable(com_position, support_polygon):
        print("Balance instability detected")
        return False
    return True
```

### Walking Pattern Debugging
1. **Step parameter validation**:
   - Verify step length, width, and height
   - Check double support phase timing
   - Monitor ground clearance

2. **Foot contact analysis**:
   - Monitor contact forces during gait cycle
   - Verify proper heel-strike and toe-off
   - Check for slipping during stance phase

3. **Joint trajectory validation**:
   - Ensure smooth joint transitions
   - Check for excessive velocities/accelerations
   - Verify joint limits are respected

## Performance Optimization

### Reducing Computational Load
1. **Simplify collision geometries** where possible
2. **Reduce unnecessary contact points**
3. **Optimize update rates for sensors**
4. **Use appropriate solver parameters**

### Memory Management
1. **Monitor memory usage over time**
2. **Check for memory leaks in plugins**
3. **Optimize model complexity**
4. **Use instancing for repeated objects**

## Troubleshooting Checklist

### Before Running Simulation
- [ ] All URDF links have mass and inertia defined
- [ ] Collision geometries match intended contact surfaces
- [ ] Joint limits are physically realistic
- [ ] Physics parameters are appropriately set
- [ ] Model is properly scaled (meters)

### During Simulation
- [ ] Monitor real-time factor (should be close to 1.0)
- [ ] Check for object penetration or falling through surfaces
- [ ] Verify joint behavior matches expectations
- [ ] Monitor for numerical instabilities
- [ ] Validate sensor outputs

### After Simulation
- [ ] Verify results match expected behavior
- [ ] Check energy conservation where applicable
- [ ] Validate that constraints are satisfied
- [ ] Assess computational performance

## Common Fixes

### For Penetration Issues
```xml
<!-- Increase contact stiffness -->
<surface>
  <contact>
    <ode>
      <kp>10000000</kp>  <!-- Very high stiffness -->
      <kd>1000</kd>      <!-- Adequate damping -->
    </ode>
  </contact>
</surface>
```

### For Instability Issues
```xml
<!-- Conservative physics parameters -->
<physics type="ode">
  <max_step_size>0.0005</max_step_size>  <!-- Smaller time step -->
  <real_time_factor>0.5</real_time_factor> <!-- Slower simulation -->
  <ode>
    <solver>
      <iters>1000</iters>  <!-- More iterations -->
    </solver>
  </ode>
</physics>
```

### For Humanoid Balance
```xml
<!-- Proper mass distribution -->
<link name="torso">
  <inertial>
    <mass value="10.0"/>  <!-- Substantial mass for stability -->
    <inertia ixx="0.2" ixy="0.0" ixz="0.0"
             iyy="0.3" iyz="0.0" izz="0.1"/>
  </inertial>
</link>
```

## Advanced Debugging Techniques

### Model Reduction for Debugging
- Create simplified models for testing
- Isolate individual joints or limbs
- Use planar rather than 3D models when possible

### Parameter Sweep Testing
- Systematically vary key parameters
- Monitor simulation behavior changes
- Identify optimal parameter ranges

### Comparison with Analytical Solutions
- Create simple scenarios with known solutions
- Compare simulation results with theory
- Identify systematic errors

## Resources for Further Debugging

### Gazebo Documentation
- [Gazebo Physics Tutorials](http://gazebosim.org/tutorials?tut=physics)
- [Troubleshooting Guide](http://gazebosim.org/tutorials?tut=troubleshooting)

### ROS 2 Integration
- [gazebo_ros_pkgs Documentation](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [Robot State Publisher Issues](https://github.com/ros/robot_state_publisher)

### Physics Simulation Research
- [Stable and Efficient Simulation of Humanoid Robots](https://ieeexplore.ieee.org/document/8206275)
- [Physics Simulation Best Practices](https://arxiv.org/abs/1802.09463)

## Summary

Effective debugging of physics simulation requires a systematic approach combining visualization tools, parameter verification, and validation techniques. For humanoid robots, special attention must be paid to balance, joint limits, and multi-body dynamics. By following the approaches outlined in this section, you can identify and resolve most common physics simulation issues.

---

**Next**: [Assessment Questions](./assessment.md) or [Chapter Summary](./summary.md)