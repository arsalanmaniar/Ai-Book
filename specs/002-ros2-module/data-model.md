# Data Model: Module 1 "The Robotic Nervous System (ROS 2)"

## Overview
This document defines the conceptual data models relevant to Module 1 "The Robotic Nervous System (ROS 2)". Since this is an educational module, the "data model" refers to the key concepts and structures that students need to understand.

## Core Entities

### ROS 2 Node
- **Definition**: A process that performs computation and communicates with other nodes
- **Key Properties**:
  - Name: Unique identifier within the ROS graph
  - Namespace: Organizational grouping for nodes
  - Parameters: Configurable values that can be set at runtime
- **Relationships**: Connects to other nodes through topics, services, and actions

### Topic
- **Definition**: Named bus over which nodes exchange messages using publisher-subscriber pattern
- **Key Properties**:
  - Name: Unique identifier for the communication channel
  - Message Type: Defines the structure of data being transmitted
  - Quality of Service (QoS): Configurable policies for reliability and performance
- **Relationships**:
  - One publisher can have multiple subscribers
  - Multiple publishers can write to the same topic (aggregation)

### Service
- **Definition**: Synchronous request-response communication pattern between nodes
- **Key Properties**:
  - Name: Unique identifier for the service
  - Request Type: Structure of data sent from client
  - Response Type: Structure of data returned from server
- **Relationships**: One service server serves multiple clients

### Message
- **Definition**: Data structure used for communication between nodes
- **Key Properties**:
  - Fields: Named data elements with specific types
  - Type Definition: Standardized format (e.g., std_msgs/String, sensor_msgs/LaserScan)
- **Relationships**: Used by topics (publisher/subscriber) and services (request/response)

### URDF Robot Model
- **Definition**: XML description of robot structure including kinematics, dynamics, and visual properties
- **Key Properties**:
  - Links: Rigid body elements with mass, inertia, and visual/collision properties
  - Joints: Connections between links with type (revolute, prismatic, etc.) and limits
  - Materials: Visual appearance properties
- **Relationships**: Links connected by joints form kinematic chains

### Quality of Service (QoS)
- **Definition**: Set of policies that define communication behavior in ROS 2
- **Key Properties**:
  - Reliability: Best effort vs. reliable delivery
  - Durability: Volatile vs. transient local (latching)
  - History: Keep last N messages vs. keep all
  - Deadline: Maximum time between consecutive messages
- **Relationships**: Applied to topics and services to control communication behavior

## State Transitions

### Node Lifecycle
1. **Unconfigured** → **Inactive**: Node created but not yet configured
2. **Inactive** → **Active**: Node activated and ready to process data
3. **Active** → **Inactive**: Node deactivated
4. **Inactive** → **Finalized**: Node cleaned up and destroyed

### Communication States
1. **Disconnected**: No connection established between nodes
2. **Connecting**: Attempting to establish connection
3. **Connected**: Active communication channel established
4. **Error**: Communication failure occurred

## Validation Rules

### For Topic Communication
- Message types must match between publisher and subscriber
- Topic names must follow ROS naming conventions
- QoS profiles should be compatible between publisher and subscriber

### For Service Communication
- Request and response types must match between client and server
- Service names must be unique within the ROS graph

### For URDF Models
- All joints must connect exactly two links
- At least one link must be designated as the root (no parent)
- Joint limits must be physically reasonable
- Mass properties must be positive

## Relationships

```
ROS 2 Node 1 --(publishes to)--> Topic <--(subscribes to)-- ROS 2 Node 2
       |                                |
       |(provides)                      |(requests)
       v                                v
  Service Server <--(handles)--- Service Request/Response ---(from)--> Service Client

URDF Model
├── Link 1 (root)
│   ├── Joint 1 (connects to Link 2)
│   └── Visual Properties
└── Link 2
    ├── Joint 2 (connects to Link 3)
    └── Collision Properties
```