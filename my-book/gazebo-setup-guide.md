# Gazebo Installation and Setup Guide

## System Requirements

- Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- At least 4GB RAM (8GB+ recommended)
- Multi-core processor
- Graphics card with OpenGL 2.0+ support
- 5GB+ free disk space

## Installation Steps

### 1. Set up the ROS 2 GPG key and repository

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2. Install Gazebo Harmonic

```bash
sudo apt update
sudo apt install gazebo libgazebo-dev
```

### 3. Install Gazebo ROS packages

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

### 4. Verify Installation

Test that Gazebo launches correctly:

```bash
gazebo
```

You should see the Gazebo interface with a default environment.

## Post-Installation Configuration

### Environment Setup

Add the following to your `~/.bashrc` file:

```bash
source /opt/ros/humble/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo
```

Then reload your bash configuration:

```bash
source ~/.bashrc
```

## Basic Gazebo Concepts

### Worlds
- Gazebo simulations take place in "worlds"
- Worlds define the environment, physics properties, and initial objects
- Stored as `.world` files in SDF (Simulation Description Format)

### Models
- 3D objects that can be placed in the world
- Can be robots, obstacles, or environmental elements
- Stored as `.sdf` files or URDF files converted to SDF

### Plugins
- Extend Gazebo functionality
- Can control robot behavior, add sensors, or modify physics
- Written in C++ or Python

## First Simulation

Try running a basic simulation:

```bash
gazebo --verbose /usr/share/gazebo-11/worlds/empty.world
```

## Troubleshooting

### Common Issues:

1. **Gazebo fails to start with graphics errors:**
   - Ensure your graphics drivers are up to date
   - Try running with software rendering: `export LIBGL_ALWAYS_SOFTWARE=1`

2. **Missing models or resources:**
   - Check that GAZEBO_MODEL_PATH is set correctly
   - Verify ROS 2 environment is sourced

3. **Performance issues:**
   - Reduce physics update rate in world files
   - Close other graphics-intensive applications