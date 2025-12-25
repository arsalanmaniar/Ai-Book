# Unity Installation and Setup Guide

**Feature**: Module 2 "The Digital Twin (Gazebo & Unity)"
**Created**: 2025-12-20
**Last Updated**: 2025-12-20

## Overview

This guide provides step-by-step instructions for installing and setting up Unity for use with the Digital Twin simulation environment. This setup will enable high-fidelity rendering and human-robot interaction capabilities.

## Prerequisites

Before beginning the installation, ensure you have:

- Windows 10 or later, macOS 10.14 or later, or Ubuntu 20.04 LTS or later
- At least 20 GB of free disk space
- 8 GB RAM minimum (16 GB recommended)
- A graphics card with DirectX 10 or OpenGL 3.3 support
- Internet connection for downloading Unity Hub and packages

## Installing Unity Hub

Unity Hub is the recommended way to manage Unity installations and projects.

### Step 1: Download Unity Hub

1. Go to [Unity's official website](https://unity.com/)
2. Navigate to the "Products" section and select "Unity Hub"
3. Download the appropriate version for your operating system
4. Run the installer and follow the on-screen instructions

### Step 2: Create Unity Account (if needed)

1. If you don't have a Unity ID, click "Create Account" in Unity Hub
2. Fill in the required information
3. Verify your email address if prompted

## Installing Unity 2022.3 LTS

### Step 1: Open Unity Hub

Launch Unity Hub from your applications menu or desktop shortcut.

### Step 2: Install Unity Editor

1. Click the "Installs" tab in Unity Hub
2. Click the "Add" button
3. Select "Unity 2022.3.22f1" (or the latest 2022.3 LTS version)
4. In the installer window, ensure the following modules are selected:
   - **Unity Android Build Support** (for mobile deployment, optional)
   - **Unity iOS Build Support** (for mobile deployment, optional)
   - **Unity Linux Build Support** (for Linux deployment, optional)
   - **Unity macOS Build Support** (for macOS deployment, optional)
   - **Unity Package Manager (UPM)**
   - **Visual Studio Tools for Unity** (Windows) or **Visual Studio Code Integration** (Mac/Linux)
5. Click "Done" to begin the installation

### Step 3: Install Additional Packages

Once Unity is installed:

1. Open Unity Hub and go to the "Projects" tab
2. Click "New Project" and create a test project to verify installation
3. Open the test project in Unity Editor
4. In Unity Editor, go to Window → Package Manager
5. Install the following packages:
   - **Universal Render Pipeline (URP)** or **High Definition Render Pipeline (HDRP)**
   - **Cinemachine** (for camera control)
   - **Unity Physics** (if not already included)
   - **ProBuilder** (for quick 3D modeling)

## Configuring Unity for Robotics Simulation

### Step 1: Set up Physics Settings

1. In Unity Editor, go to Edit → Project Settings → Physics
2. Configure the following settings for robotics simulation:
   - **Gravity**: (0, -9.81, 0) to match Earth's gravity
   - **Default Material**: Create a default material with appropriate friction for robot-world interaction
   - **Layer Collision Matrix**: Configure collision layers to match your simulation needs

### Step 2: Configure Rendering Pipeline

1. If using URP, create a new URP Asset (Assets → Create → Rendering → Universal Render Pipeline → Pipeline Asset)
2. In Project Settings → Graphics, assign the new URP Asset
3. Configure the render pipeline for your visual quality requirements

### Step 3: Set up ROS Integration (Optional)

For ROS 2 integration with Unity:

1. Download the Unity Robotics Hub from the Unity Asset Store or GitHub
2. Import the ROS TCP Connector package
3. Configure the connection settings to match your ROS 2 setup

## Verification

To verify your Unity installation:

1. Create a new 3D project in Unity Hub
2. In the new project, create a simple scene:
   - Add a cube (GameObject → 3D Object → Cube) at position (0, 0, 0)
   - Add a sphere (GameObject → 3D Object → Sphere) at position (2, 1, 0)
   - Add a directional light (GameObject → Light → Directional Light)
   - Add a camera (GameObject → Camera) if not already present
3. Play the scene (click the Play button)
4. Verify that you can see the objects and that the scene renders correctly

## Troubleshooting

### Common Issues

- **Graphics driver issues**: Update your graphics drivers to the latest version
- **Insufficient disk space**: Free up disk space before installing Unity
- **Installation fails**: Run the installer as administrator (Windows) or with sudo (Linux)
- **Performance issues**: Lower graphics quality settings in Edit → Project Settings → Quality

### Performance Optimization for Robotics Simulation

For optimal performance with robotics simulations:

1. Use appropriate Level of Detail (LOD) groups for complex robot models
2. Use occlusion culling for large environments
3. Optimize materials and shaders for your target platform
4. Use appropriate lighting settings (baked vs real-time)

## Next Steps

Once Unity is properly installed and configured:

1. Create your first humanoid robot model following the modeling guidelines
2. Set up a basic scene with lighting and environment
3. Begin implementing the Unity-based rendering and interaction content from Chapter 5
4. Integrate with Gazebo physics simulation using the provided interfaces

## Additional Resources

- [Unity Manual](https://docs.unity3d.com/Manual/index.html)
- [Unity Scripting API](https://docs.unity3d.com/ScriptReference/)
- [Unity Robotics Hub Documentation](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [Universal Render Pipeline Documentation](https://docs.unity3d.com/Packages/com.unity.render-pipelines.universal@latest)