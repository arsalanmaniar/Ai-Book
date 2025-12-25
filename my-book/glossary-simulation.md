---
title: Glossary - Simulation Terms
sidebar_label: Simulation Glossary
---

# Glossary: Simulation Terms

**Module**: Module 2 "The Digital Twin (Gazebo & Unity)"
**Document**: Glossary of Simulation Terms
**Created**: 2025-12-20
**Last Updated**: 2025-12-20

## A

**Acceleration Structure**: Data structure used in physics engines to optimize collision detection by reducing the number of potential collision pairs to check.

**Actuator**: A component of a robot that converts control signals into physical motion, such as motors or pneumatic/hydraulic systems.

**Articulated Body**: A mechanical system composed of multiple rigid bodies connected by joints, commonly used to model robotic arms or humanoid robots.

## B

**Bounding Volume**: A geometric volume that completely encloses an object, used for efficient collision detection and visibility testing (e.g., bounding boxes, spheres).

## C

**Collision Detection**: The computational problem of detecting when two or more bodies come into contact with each other during simulation.

**Collision Mesh**: A simplified geometric representation of an object used specifically for collision detection, often simpler than the visual mesh.

**Contact Point**: The specific location where two objects make contact during a collision, used for calculating contact forces.

**Coordinate Frame**: A system of reference that defines positions and orientations in 3D space, essential for multi-sensor and multi-robot systems.

**Cross-Validation**: The process of validating simulation results by comparing them with real-world data or other simulation models.

## D

**Degrees of Freedom (DOF)**: The number of independent parameters that define the configuration or state of a mechanical system.

**Dynamics**: The branch of mechanics concerned with the motion of bodies under the action of forces.

**Dynamical System**: A system in which a function describes the time dependence of a point in a geometrical space, fundamental to physics simulation.

## E

**Euler Integration**: A first-order numerical procedure for solving ordinary differential equations with a given initial value, used in basic physics simulation.

**Extended Kalman Filter (EKF)**: A nonlinear version of the Kalman filter that linearizes the system model around the current estimate.

## F

**Forward Kinematics**: The use of kinematic equations to compute the position of the end-effector from specified values of joint parameters.

**Friction Model**: Mathematical representation of the resistive force that opposes the relative motion of two surfaces in contact.

## G

**Gazebo**: An open-source 3D robotics simulator that provides accurate and efficient simulation of robots and their environments.

**Gaussian Noise**: Statistical noise having a probability density function equal to that of the normal distribution, commonly used in sensor simulation.

**Gravity Compensation**: The process of accounting for gravitational forces in robot control and simulation.

## H

**Haptic Feedback**: The use of touch and motion feedback to provide information to a user, important in human-robot interaction.

**Hessian Matrix**: In optimization, a square matrix of second-order partial derivatives of a scalar-valued function, used in physics optimization.

## I

**Inertial Measurement Unit (IMU)**: An electronic device that measures and reports a body's specific force, angular rate, and sometimes magnetic field.

**Inverse Dynamics**: The computation of forces and torques required to achieve a given motion, used in robot control.

**Iterative Solver**: A method that approaches the solution to a problem through successive approximations, commonly used in physics simulation.

## J

**Joint**: A connection between two or more links in a mechanical system that allows relative motion between them.

**Joint Limits**: Physical or software-imposed constraints on the range of motion of a robotic joint.

## K

**Kalman Filter**: An algorithm that uses a series of measurements observed over time to produce estimates of unknown variables.

**Kinematics**: The branch of mechanics concerned with the motion of objects without reference to the forces that cause the motion.

## L

**Laser Range Finder (LIDAR)**: A surveying method that measures distance to a target by illuminating it with pulsed laser light.

**Linear Complementarity Problem (LCP)**: A mathematical optimization problem that arises in contact mechanics and friction simulation.

**Logarithmic Spiral**: A self-similar spiral curve that often appears in nature, sometimes used in path planning algorithms.

## M

**Mass Matrix**: A matrix that relates the acceleration of a system to the forces acting on it in multibody dynamics.

**Model Predictive Control (MPC)**: An advanced method of process control that uses a model of the system to predict future behavior.

**Multi-body Dynamics**: The study of the motion of interconnected bodies in engineering systems, fundamental to robotics simulation.

## N

**Newton-Euler Algorithm**: A recursive method for computing the forward dynamics of a serial chain of rigid bodies.

**Noise Density**: A measure of the noise power distributed over frequency, commonly used to characterize sensor noise.

## O

**ODE (Open Dynamics Engine)**: An open-source physics engine library that simulates rigid body dynamics.

**Odometry**: The use of data from motion sensors to estimate change in position over time, commonly used in robotics navigation.

**Optimization**: The mathematical procedure for finding the best solution to a problem according to some criterion.

## P

**PID Controller**: A control loop feedback mechanism widely used in industrial control systems and robotics applications.

**Physically Based Rendering (PBR)**: A method of shading and rendering that provides more accurate representation of how light interacts with surfaces.

**Point Cloud**: A set of data points in space, commonly produced by 3D scanners and depth cameras.

**Pose**: The position and orientation of a robot or object in space, typically represented by a 6-DOF transformation.

**Prismatic Joint**: A type of joint that allows linear sliding movement between two bodies along a single axis.

## Q

**Quaternion**: A mathematical construct used to represent rotations in 3D space, avoiding the gimbal lock problem of Euler angles.

## R

**Ray Tracing**: A rendering technique for generating realistic images by tracing the path of light as pixels in an image plane.

**Rigid Body**: An idealization of a solid body in which deformation is neglected, fundamental to physics simulation.

**Robot Operating System (ROS)**: Flexible framework for writing robot software, providing services designed for a heterogeneous computer cluster.

**ROS Package**: A modular collection of code that serves a specific purpose in the Robot Operating System framework.

**ROS Node**: A process that performs computation in the Robot Operating System framework.

**ROS Topic**: A named bus over which nodes exchange messages in the Robot Operating System framework.

**ROS Message**: A data structure exchanged between nodes in the Robot Operating System framework.

**ROS Service**: A synchronous request/reply mechanism for remote procedure calls in the Robot Operating System framework.

## S

**Sensor Fusion**: The process of combining data from multiple sensors to achieve better accuracy and reliability than could be achieved by any individual sensor.

**Servo Motor**: A rotary actuator that allows for precise control of angular position, velocity, and acceleration.

**Simulation Fidelity**: The degree to which a simulation accurately represents the real-world system it is modeling.

**State Estimation**: The process of estimating the internal state of a dynamic system from measurements of its inputs and outputs.

**State Space**: The set of all possible states of a dynamical system, fundamental to control theory.

**Stereographic Projection**: A particular mapping that projects a sphere onto a plane, sometimes used in sensor modeling.

## T

**Temporal Resolution**: The smallest time interval that can be resolved by a sensor or simulation system.

**Torque Control**: A method of controlling a motor by controlling the rotational force it applies rather than its position or velocity.

**Transformation Matrix**: A 4x4 matrix used to represent position, orientation, and scale in 3D space.

**Trajectory Planning**: The process of determining a path for a robot to follow from a start state to a goal state.

## U

**Unscented Kalman Filter (UKF)**: A nonlinear extension of the Kalman filter that uses a deterministic sampling approach.

**URDF (Unified Robot Description Format)**: An XML format for representing a robot model in ROS.

**Unity**: A cross-platform game engine commonly used for creating 3D simulations and visualizations in robotics.

**Unity-Gazebo Integration**: The connection between Unity visualization and Gazebo physics simulation environments.

## V

**Virtual Reality (VR)**: A simulated experience that can be similar to or completely different from the real world, used in robotics training and visualization.

**Visual Servoing**: The process of controlling a robot using visual feedback from one or more cameras.

## W

**World Coordinate System**: A fixed reference frame used to define positions and orientations in a simulation environment.

**Wrench**: A mathematical representation combining forces and torques acting on a rigid body.

## X, Y, Z

**XYZ Coordinate System**: A three-dimensional coordinate system using X, Y, and Z axes to define positions in space.

**Zero-Order Hold**: A mathematical model of the real-world process of converting a discrete-time signal to a continuous-time signal, used in digital control systems.

---

**Next**: [Glossary of Robotics Terms](./glossary-robotics.md) or [Chapter 4: Gazebo Physics](./chapter-4-gazebo-physics/introduction.md)