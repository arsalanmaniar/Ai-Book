# API Contract: Vision-Language-Action (VLA) System

## Overview
This document defines the API contracts for the Vision-Language-Action (VLA) system that integrates Large Language Models with robotics, processes voice commands, and maps them to ROS 2 actions.

## Voice Processing Service

### POST /voice/process
Process a voice command and return an action sequence.

**Request:**
```json
{
  "audio_data": "base64_encoded_audio",
  "language": "en-US",
  "user_intent": "navigation",
  "context": {
    "robot_state": "idle",
    "environment": "indoor",
    "available_actions": ["move_forward", "turn_left", "turn_right", "stop"]
  }
}
```

**Response:**
```json
{
  "transcription": "move forward 1 meter",
  "confidence": 0.92,
  "parsed_intent": "navigation.move",
  "action_sequence": [
    {
      "action_name": "move_forward",
      "parameters": {"distance": 1.0, "speed": 0.5},
      "estimated_time": 5.0
    }
  ],
  "metadata": {
    "processing_time": 1.2,
    "model_used": "gpt-3.5-turbo"
  }
}
```

**Errors:**
- 400: Invalid audio format or missing parameters
- 408: Timeout during processing
- 500: Internal processing error

## LLM Interface Service

### POST /llm/process
Process natural language input and generate action sequences.

**Request:**
```json
{
  "text": "pick up the red block and place it on the table",
  "context": {
    "robot_capabilities": ["arm_control", "gripper_control", "navigation"],
    "environment_objects": ["red_block", "table", "shelf"],
    "robot_position": {"x": 0.0, "y": 0.0, "z": 0.0}
  },
  "options": {
    "max_actions": 10,
    "response_format": "action_sequence"
  }
}
```

**Response:**
```json
{
  "original_text": "pick up the red block and place it on the table",
  "parsed_tasks": [
    {"description": "navigate to red block", "type": "navigation"},
    {"description": "pick up red block", "type": "manipulation"},
    {"description": "navigate to table", "type": "navigation"},
    {"description": "place block on table", "type": "manipulation"}
  ],
  "action_sequence": [
    {
      "action_name": "find_object",
      "parameters": {"object_type": "red_block"},
      "requires_feedback": true
    },
    {
      "action_name": "navigate_to",
      "parameters": {"target_object": "red_block"},
      "requires_feedback": true
    },
    {
      "action_name": "grasp_object",
      "parameters": {"object_id": "red_block_123"},
      "requires_feedback": true
    }
  ],
  "estimated_completion_time": 60.0
}
```

**Errors:**
- 400: Invalid request format
- 422: Unprocessable content (unclear command)
- 500: LLM service error

## ROS 2 Action Mapper

### POST /actions/map
Map high-level action requests to specific ROS 2 action calls.

**Request:**
```json
{
  "high_level_action": "navigate_to",
  "parameters": {
    "target_location": {
      "x": 2.0,
      "y": 3.0,
      "theta": 0.0
    },
    "frame": "map"
  },
  "robot_id": "humanoid_001"
}
```

**Response:**
```json
{
  "mapped_action": {
    "action_server": "/move_base_flex/exe_path",
    "action_type": "mbf_msgs/ExePathAction",
    "goal": {
      "target_pose": {
        "header": {
          "frame_id": "map"
        },
        "pose": {
          "position": {"x": 2.0, "y": 3.0, "z": 0.0},
          "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        }
      }
    }
  },
  "estimated_duration": 15.0
}
```

## Cognitive Planning Service

### POST /planning/generate
Generate a complete action plan for a complex task.

**Request:**
```json
{
  "task_description": "Clean the living room by picking up objects and placing them in designated locations",
  "environment_map": "map_id_456",
  "robot_capabilities": ["navigation", "manipulation", "object_recognition"],
  "constraints": {
    "max_time": 300, // 5 minutes
    "safety_radius": 0.5,
    "preferred_objects": ["book", "cup", "toy"]
  }
}
```

**Response:**
```json
{
  "plan_id": "plan_789",
  "status": "generated",
  "actions": [
    {
      "id": 1,
      "type": "navigation",
      "description": "navigate to living room entrance",
      "estimated_duration": 10.0,
      "preconditions": [],
      "effects": ["robot_at_location"]
    },
    {
      "id": 2,
      "type": "object_detection",
      "description": "scan area for objects to clean",
      "estimated_duration": 5.0,
      "preconditions": ["robot_at_location"],
      "effects": ["objects_identified"]
    }
  ],
  "estimated_total_time": 120.0,
  "confidence": 0.85
}
```

## Status and Monitoring

### GET /status
Get the current status of the VLA system.

**Response:**
```json
{
  "system_status": "operational",
  "voice_processor": "ready",
  "llm_service": "ready",
  "ros2_bridge": "connected",
  "active_robot": "humanoid_001",
  "active_plan": null,
  "last_command_time": "2025-12-21T12:00:00Z",
  "error_count": 0,
  "uptime": 3600 // seconds
}
```

### GET /status/robot/{robot_id}
Get detailed status for a specific robot.

**Response:**
```json
{
  "robot_id": "humanoid_001",
  "status": "idle",
  "battery_level": 0.85,
  "current_action": null,
  "position": {
    "x": 0.0,
    "y": 0.0,
    "theta": 0.0
  },
  "available_actions": ["move_forward", "turn_left", "turn_right", "speak", "grasp", "release"],
  "last_update": "2025-12-21T12:00:00Z"
}
```