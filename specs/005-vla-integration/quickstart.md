# Quickstart Guide: Module 4 "Vision-Language-Action (VLA)" for Physical AI & Humanoid Robotics Textbook

## Overview
This quickstart guide provides the essential steps for students to begin working with Vision-Language-Action (VLA) systems, integrating Large Language Models with robotics, and building voice-to-action pipelines.

## Prerequisites
- Python 3.8+ installed
- ROS 2 Humble Hawksbill installed and configured
- Access to OpenAI API key (or local LLM setup)
- Gazebo simulation environment
- Basic understanding of ROS 2 concepts (topics, services, actions)

## Setup Steps

### 1. Environment Setup
```bash
# Create a new ROS 2 workspace for VLA examples
mkdir -p ~/vla_ws/src
cd ~/vla_ws

# Install required Python packages
pip install openai speech-recognition pyaudio numpy

# Build the workspace
colcon build
source install/setup.bash
```

### 2. OpenAI API Configuration
```bash
# Set your OpenAI API key as an environment variable
export OPENAI_API_KEY="your-api-key-here"

# For local LLM alternatives, install transformers
pip install transformers torch
```

### 3. Voice Recognition Setup
```bash
# Install Whisper for local speech recognition
pip install openai-whisper

# Test audio input
python -c "import speech_recognition as sr; print('Speech recognition available')"
```

### 4. ROS 2 Action Server Setup
```bash
# Navigate to your workspace
cd ~/vla_ws/src

# Create the VLA package
ros2 pkg create --build-type ament_python vla_interfaces

# Create action definition files for robot actions
# (Examples will be provided in the exercises)
```

## Basic VLA Pipeline Example

### 1. Simple Voice Command Processing
```python
# Example voice_processor.py
import speech_recognition as sr
from openai import OpenAI

def process_voice_command():
    # Initialize recognizer
    r = sr.Recognizer()

    # Listen for command
    with sr.Microphone() as source:
        print("Listening for command...")
        audio = r.listen(source)

    # Convert speech to text
    command_text = r.recognize_google(audio)
    print(f"Heard: {command_text}")

    # Process with LLM to generate action sequence
    client = OpenAI()
    response = client.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[{
            "role": "user",
            "content": f"Convert this command to a robot action sequence: {command_text}"
        }]
    )

    return response.choices[0].message.content
```

### 2. ROS 2 Action Client Example
```python
# Example action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

class VLAActionClient(Node):
    def __init__(self):
        super().__init__('vla_action_client')
        # Initialize action client for specific robot actions
        self._action_client = ActionClient(
            self,
            # Your action type here
            'action_name'
        )
```

## Running the Examples

### 1. Launch the Simulation Environment
```bash
# Start Gazebo with a humanoid robot
ros2 launch vla_examples simulation.launch.py

# In another terminal, run the voice processing node
ros2 run vla_examples voice_processor_node
```

### 2. Test Voice Commands
Speak simple commands like:
- "Move forward 1 meter"
- "Turn left"
- "Pick up the red block"
- "Navigate to the kitchen"

## Key Concepts to Master

1. **VLA Pipeline Flow**: Voice → Speech-to-Text → LLM Processing → Action Mapping → Robot Execution
2. **ROS 2 Action Patterns**: Understanding goal, feedback, and result patterns
3. **LLM Prompt Engineering**: Crafting effective prompts for robot task interpretation
4. **Error Handling**: Managing cases where commands cannot be processed or executed

## Next Steps
1. Complete the exercises in `chapter-7-vla-integration/exercises.md`
2. Explore advanced integration techniques in Chapter 8
3. Begin the capstone project in Chapter 9

## Troubleshooting
- If voice recognition fails, check microphone permissions and audio levels
- If ROS 2 actions don't execute, verify action server is running
- If LLM responses are incorrect, refine your prompt engineering techniques