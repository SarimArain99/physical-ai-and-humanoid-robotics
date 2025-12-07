---
sidebar_position: 13
---

# Conversational Robotics and Capstone Integration

This chapter focuses on integrating conversational AI with robotic systems, bringing together all the components learned in previous weeks to create an AI-native humanoid robot capable of natural human interaction.

## Learning Objectives

By the end of this week, you will be able to:
- Integrate GPT models with robotic systems for conversational AI
- Implement speech recognition and natural language understanding for robotics
- Design multimodal interaction systems combining speech, vision, and action
- Create a complete capstone project: autonomous humanoid with conversational AI
- Deploy and evaluate the integrated system

## Prerequisites

- Understanding of all previous modules (Weeks 1-12)
- Knowledge of AI-robot brains and VLA systems
- Familiarity with sensor fusion and humanoid control
- Understanding of NVIDIA Isaac platform and ROS 2
- Experience with multimodal interaction systems

## Introduction to Conversational Robotics

Conversational robotics combines natural language processing, speech recognition, and robotic action execution to create robots that can engage in natural human-like conversations while performing physical tasks. This represents the convergence of:

- **Natural Language Processing**: Understanding and generating human language
- **Speech Recognition/Synthesis**: Converting speech to text and vice versa
- **Robot Control**: Executing actions based on language commands
- **Perception Systems**: Understanding context through vision and sensors

### Key Components of Conversational Robotics

1. **Speech Recognition**: Converting human speech to text
2. **Natural Language Understanding**: Interpreting the meaning of commands
3. **Dialog Management**: Maintaining conversation context and flow
4. **Action Planning**: Converting language to robot actions
5. **Speech Synthesis**: Converting robot responses to speech
6. **Multimodal Integration**: Combining speech, vision, and action

## Speech Recognition and Natural Language Understanding

### Advanced Speech Recognition Pipeline

```python
import speech_recognition as sr
import openai
import asyncio
import threading
from typing import Dict, List, Optional
from dataclasses import dataclass

@dataclass
class SpeechRecognitionResult:
    """Result from speech recognition with confidence and context"""
    text: str
    confidence: float
    timestamp: float
    language: str = 'en-US'

class AdvancedSpeechRecognizer:
    """Advanced speech recognition system for conversational robotics"""
    def __init__(self, openai_api_key: str = None):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Initialize OpenAI if API key provided
        if openai_api_key:
            openai.api_key = openai_api_key
            self.use_llm_enhancement = True
        else:
            self.use_llm_enhancement = False

        # Speech recognition parameters
        self.energy_threshold = 4000
        self.dynamic_energy_threshold = True
        self.pause_threshold = 0.8  # Seconds of silence before phrase is considered complete
        self.phrase_time_limit = 10.0  # Maximum seconds for a phrase

        # Wake word detection
        self.wake_words = ['robot', 'hey robot', 'hello robot', 'assistant']
        self.is_listening = False

        # Recognition history for context
        self.recognition_history = []

    def listen_for_wake_word(self, timeout: float = 10.0) -> bool:
        """Listen for wake word to activate the robot"""
        try:
            with self.microphone as source:
                print("Listening for wake word...")
                audio = self.recognizer.listen(source, timeout=timeout, phrase_time_limit=5.0)

            # Recognize speech
            text = self.recognizer.recognize_google(audio).lower()

            # Check for wake words
            for wake_word in self.wake_words:
                if wake_word in text:
                    print(f"Wake word detected: {wake_word}")
                    return True

            return False

        except sr.WaitTimeoutError:
            print("No wake word detected within timeout")
            return False
        except sr.UnknownValueError:
            print("Could not understand audio")
            return False
        except sr.RequestError as e:
            print(f"Speech recognition error: {e}")
            return False

    def listen_for_command(self, timeout: float = 10.0) -> Optional[SpeechRecognitionResult]:
        """Listen for and recognize a command after wake word"""
        try:
            with self.microphone as source:
                print("Listening for command...")
                audio = self.recognizer.listen(source, timeout=timeout, phrase_time_limit=self.phrase_time_limit)

            # Recognize speech
            text = self.recognizer.recognize_google(audio)
            confidence = 0.9  # Google's API doesn't return confidence, so we estimate

            # Enhance with LLM if available
            if self.use_llm_enhancement:
                enhanced_text = self.enhance_command_with_llm(text)
                result = SpeechRecognitionResult(
                    text=enhanced_text,
                    confidence=confidence,
                    timestamp=time.time()
                )
            else:
                result = SpeechRecognitionResult(
                    text=text,
                    confidence=confidence,
                    timestamp=time.time()
                )

            # Store in history
            self.recognition_history.append(result)

            return result

        except sr.WaitTimeoutError:
            print("No command detected within timeout")
            return None
        except sr.UnknownValueError:
            print("Could not understand command")
            return None
        except sr.RequestError as e:
            print(f"Speech recognition error: {e}")
            return None

    def enhance_command_with_llm(self, raw_command: str) -> str:
        """Use LLM to enhance and clarify natural language commands"""
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": "You are a command clarifier for a robot. "
                                   "Clarify ambiguous natural language commands "
                                   "into more specific, actionable instructions. "
                                   "Preserve the user's intent while making the command more precise."
                    },
                    {
                        "role": "user",
                        "content": f"Clarify this command: '{raw_command}'. "
                                   "Return only the clarified command."
                    }
                ],
                temperature=0.1,
                max_tokens=50
            )

            clarified_command = response.choices[0].message.content.strip()
            print(f"Command enhanced: '{raw_command}' -> '{clarified_command}'")
            return clarified_command

        except Exception as e:
            print(f"LLM enhancement failed: {e}")
            return raw_command  # Return original if LLM fails

    def get_contextual_command(self, command: str) -> Dict:
        """Extract context and intent from command"""
        # Use rule-based parsing for demonstration
        # In practice, this would use more sophisticated NLP

        # Identify action verbs
        action_verbs = {
            'move': ['go', 'move', 'walk', 'navigate', 'drive', 'approach', 'go to'],
            'grasp': ['pick', 'grasp', 'grab', 'take', 'lift', 'catch'],
            'place': ['put', 'place', 'set', 'drop', 'release', 'lay'],
            'turn': ['turn', 'rotate', 'pivot', 'spin', 'left', 'right'],
            'stop': ['stop', 'halt', 'pause', 'cease'],
            'find': ['find', 'locate', 'search', 'look', 'identify'],
            'greet': ['hello', 'hi', 'greetings', 'good morning', 'good afternoon'],
            'follow': ['follow', 'track', 'accompany']
        }

        # Identify objects
        objects = [
            'person', 'chair', 'table', 'cup', 'bottle', 'box',
            'door', 'window', 'plant', 'book', 'phone', 'laptop',
            'kitchen', 'bedroom', 'living room', 'office', 'hallway'
        ]

        # Parse command
        command_lower = command.lower()

        # Extract action
        action = None
        for action_type, verbs in action_verbs.items():
            for verb in verbs:
                if verb in command_lower:
                    action = action_type
                    break
            if action:
                break

        # Extract object
        detected_objects = []
        for obj in objects:
            if obj in command_lower:
                detected_objects.append(obj)

        # Extract location
        locations = ['kitchen', 'bedroom', 'living room', 'office', 'hallway', 'door', 'window']
        detected_locations = [loc for loc in locations if loc in command_lower]

        return {
            'action': action,
            'objects': detected_objects,
            'locations': detected_locations,
            'original_command': command,
            'parsed_command': self.generate_parsed_command(action, detected_objects, detected_locations)
        }

    def generate_parsed_command(self, action: str, objects: List[str], locations: List[str]) -> Dict:
        """Generate structured command from parsed elements"""
        command_structure = {
            'action': action,
            'target_objects': objects,
            'target_location': locations[0] if locations else None,
            'parameters': {}
        }

        # Add action-specific parameters
        if action == 'move':
            command_structure['parameters']['destination'] = locations[0] if locations else 'unknown'
        elif action == 'grasp':
            command_structure['parameters']['object'] = objects[0] if objects else 'unknown'
        elif action == 'greet':
            command_structure['parameters']['greeting_type'] = 'casual'

        return command_structure
```

### Natural Language Understanding with LLMs

```python
class NaturalLanguageUnderstanding:
    """Advanced NLU using LLMs for cognitive planning"""
    def __init__(self, openai_api_key: str):
        openai.api_key = openai_api_key
        self.action_library = {
            'navigation': {
                'move_to': {
                    'description': 'Move robot to specified location',
                    'parameters': ['location', 'orientation'],
                    'ros_equivalent': 'nav2_msgs/MoveToPose'
                },
                'follow_route': {
                    'description': 'Follow a predefined route',
                    'parameters': ['waypoints'],
                    'ros_equivalent': 'nav_msgs/Path'
                },
                'explore_area': {
                    'description': 'Explore an area systematically',
                    'parameters': ['boundary'],
                    'ros_equivalent': 'custom_exploration_action'
                }
            },
            'manipulation': {
                'pick_object': {
                    'description': 'Pick up an object',
                    'parameters': ['object_name', 'location'],
                    'ros_equivalent': 'manipulation_msgs/PickupObject'
                },
                'place_object': {
                    'description': 'Place object at location',
                    'parameters': ['object_name', 'location'],
                    'ros_equivalent': 'manipulation_msgs/PlaceObject'
                },
                'grasp_object': {
                    'description': 'Grasp an object',
                    'parameters': ['object_name', 'grasp_type'],
                    'ros_equivalent': 'control_msgs/GripperCommand'
                }
            },
            'perception': {
                'detect_objects': {
                    'description': 'Detect objects in the environment',
                    'parameters': ['object_types'],
                    'ros_equivalent': 'vision_msgs/Detection2DArray'
                },
                'localize_robot': {
                    'description': 'Determine robot position',
                    'parameters': [],
                    'ros_equivalent': 'geometry_msgs/PoseWithCovarianceStamped'
                },
                'map_environment': {
                    'description': 'Create map of environment',
                    'parameters': [],
                    'ros_equivalent': 'nav_msgs/OccupancyGrid'
                }
            },
            'social_interaction': {
                'greet_person': {
                    'description': 'Greet a person',
                    'parameters': ['person_id', 'greeting_type'],
                    'ros_equivalent': 'custom_greeting_action'
                },
                'follow_person': {
                    'description': 'Follow a person',
                    'parameters': ['person_id', 'following_distance'],
                    'ros_equivalent': 'custom_follow_action'
                },
                'answer_question': {
                    'description': 'Answer a question',
                    'parameters': ['question', 'context'],
                    'ros_equivalent': 'custom_dialog_action'
                }
            }
        }

    def understand_command(self, command: str, robot_capabilities: List[str]) -> Dict:
        """Understand natural language command and generate action sequence"""
        try:
            # Create detailed prompt for cognitive planning
            prompt = f"""
            You are an AI assistant that translates natural language commands into sequences of robotic actions.
            The robot has these capabilities: {', '.join(robot_capabilities)}

            Given this command: "{command}"

            Please break it down into a sequence of specific robotic actions.
            Each action should be from the following categories:
            - Navigation: move_to, follow_route, explore_area
            - Manipulation: pick_object, place_object, grasp_object
            - Perception: detect_objects, localize_robot, map_environment
            - Social Interaction: greet_person, follow_person, answer_question

            Return the sequence as a JSON list of action dictionaries with format:
            {{
                "action": "action_name",
                "parameters": {{"param_name": "param_value"}},
                "description": "Brief description of the action",
                "dependencies": ["list", "of", "required", "conditions"],
                "confidence": 0.0-1.0
            }}

            Be specific about locations, objects, and conditions.
            If the command is a question, return actions to gather information and answer.
            If the command is a request to perform a task, return actions to accomplish it.
            """

            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1,
                max_tokens=800
            )

            # Parse the response
            response_text = response.choices[0].message.content.strip()

            # Extract JSON from response (in case it includes additional text)
            import json
            import re

            # Look for JSON array in response
            json_match = re.search(r'\[(.*?)\]', response_text, re.DOTALL)

            if json_match:
                json_str = '[' + json_match.group(1) + ']'
                action_sequence = json.loads(json_str)

                # Validate action sequence
                validated_sequence = self.validate_action_sequence(action_sequence)
                return {
                    'success': True,
                    'action_sequence': validated_sequence,
                    'original_command': command,
                    'confidence': self.calculate_overall_confidence(validated_sequence)
                }
            else:
                # If no JSON found, try to parse the whole response
                return self.parse_simple_response(response_text, command)

        except Exception as e:
            print(f"Error in NLU: {e}")
            return self.generate_fallback_plan(command)

    def validate_action_sequence(self, action_sequence: List[Dict]) -> List[Dict]:
        """Validate that the action sequence is executable"""
        validated_sequence = []

        for action in action_sequence:
            # Check if action is in library
            action_found = False
            for category, actions in self.action_library.items():
                if action['action'] in actions:
                    action_found = True
                    break

            if action_found:
                # Add default parameters if missing
                if 'parameters' not in action:
                    action['parameters'] = {}
                if 'description' not in action:
                    action['description'] = f"Execute {action['action']}"
                if 'dependencies' not in action:
                    action['dependencies'] = []
                if 'confidence' not in action:
                    action['confidence'] = 0.8  # Default confidence

                validated_sequence.append(action)
            else:
                print(f"Unknown action: {action['action']}")

        return validated_sequence

    def calculate_overall_confidence(self, action_sequence: List[Dict]) -> float:
        """Calculate overall confidence in action sequence"""
        if not action_sequence:
            return 0.0

        total_confidence = sum(action.get('confidence', 0.8) for action in action_sequence)
        return total_confidence / len(action_sequence)

    def parse_simple_response(self, response: str, original_command: str) -> Dict:
        """Parse simple response when JSON extraction fails"""
        # Fallback: simple keyword-based parsing
        command_lower = original_command.lower()

        # Determine action based on keywords
        if any(word in command_lower for word in ['move', 'go', 'navigate', 'walk']):
            action_sequence = [{
                'action': 'move_to',
                'parameters': {'location': 'default'},
                'description': f'Move to location based on command: {original_command}',
                'dependencies': [],
                'confidence': 0.6
            }]
        elif any(word in command_lower for word in ['pick', 'grasp', 'take']):
            action_sequence = [{
                'action': 'pick_object',
                'parameters': {'object_name': 'default_object'},
                'description': f'Pick up object based on command: {original_command}',
                'dependencies': ['object_detected'],
                'confidence': 0.5
            }]
        elif any(word in command_lower for word in ['hello', 'hi', 'greet']):
            action_sequence = [{
                'action': 'greet_person',
                'parameters': {'greeting_type': 'casual'},
                'description': f'Greet person based on command: {original_command}',
                'dependencies': [],
                'confidence': 0.9
            }]
        else:
            # For questions or general commands
            action_sequence = [{
                'action': 'answer_question',
                'parameters': {'question': original_command},
                'description': f'Process question/command: {original_command}',
                'dependencies': [],
                'confidence': 0.7
            }]

        return {
            'success': True,
            'action_sequence': action_sequence,
            'original_command': original_command,
            'confidence': 0.7
        }

    def generate_fallback_plan(self, command: str) -> Dict:
        """Generate a basic fallback plan when LLM fails"""
        print(f"Generating fallback plan for: {command}")

        # Simple keyword-based planning
        if 'clean' in command.lower() or 'tidy' in command.lower():
            action_sequence = [
                {
                    'action': 'detect_objects',
                    'parameters': {'object_types': ['trash', 'litter']},
                    'description': 'Detect objects that need to be cleaned',
                    'dependencies': [],
                    'confidence': 0.8
                },
                {
                    'action': 'move_to',
                    'parameters': {'location': 'nearest_trash_location'},
                    'description': 'Move to location of detected object',
                    'dependencies': ['objects_detected'],
                    'confidence': 0.7
                },
                {
                    'action': 'pick_object',
                    'parameters': {'object_name': 'detected_object'},
                    'description': 'Pick up the object',
                    'dependencies': ['at_object_location'],
                    'confidence': 0.6
                },
                {
                    'action': 'move_to',
                    'parameters': {'location': 'trash_bin'},
                    'description': 'Move to trash bin',
                    'dependencies': ['object_picked'],
                    'confidence': 0.7
                },
                {
                    'action': 'place_object',
                    'parameters': {'object_name': 'picked_object', 'location': 'trash_bin'},
                    'description': 'Place object in trash bin',
                    'dependencies': ['at_trash_bin'],
                    'confidence': 0.8
                }
            ]
        elif 'go to' in command.lower() or 'navigate to' in command.lower():
            # Extract destination from command
            import re
            location_match = re.search(r'(?:to|at|in)\s+([a-zA-Z\s]+)', command.lower())
            location = location_match.group(1).strip() if location_match else 'unknown location'

            action_sequence = [
                {
                    'action': 'localize_robot',
                    'parameters': {},
                    'description': 'Determine current robot position',
                    'dependencies': [],
                    'confidence': 0.9
                },
                {
                    'action': 'move_to',
                    'parameters': {'location': location},
                    'description': f'Move to {location}',
                    'dependencies': ['robot_localized'],
                    'confidence': 0.8
                }
            ]
        elif '?' in command:  # Question
            action_sequence = [
                {
                    'action': 'answer_question',
                    'parameters': {'question': command},
                    'description': f'Answer the question: {command}',
                    'dependencies': [],
                    'confidence': 0.7
                }
            ]
        else:
            # Default: just detect and report
            action_sequence = [
                {
                    'action': 'detect_objects',
                    'parameters': {'object_types': ['all']},
                    'description': 'Detect objects in environment',
                    'dependencies': [],
                    'confidence': 0.8
                }
            ]

        return {
            'success': True,
            'action_sequence': action_sequence,
            'original_command': command,
            'confidence': 0.6
        }
```

## Conversational AI Integration

### Dialog Management System

```python
from dataclasses import dataclass
from typing import Dict, List, Any, Optional
import time
import uuid

@dataclass
class ConversationTurn:
    """Represents a single turn in a conversation"""
    id: str
    user_input: str
    robot_response: str
    timestamp: float
    context: Dict
    action_sequence: List[Dict]

class DialogManager:
    """Manage conversation flow and context"""
    def __init__(self):
        self.conversations = {}  # Store multiple conversations
        self.current_conversation_id = None
        self.max_conversation_history = 10  # Keep last 10 turns

    def start_conversation(self) -> str:
        """Start a new conversation"""
        conversation_id = str(uuid.uuid4())
        self.conversations[conversation_id] = []
        self.current_conversation_id = conversation_id
        return conversation_id

    def add_turn(self, user_input: str, robot_response: str,
                 context: Dict, action_sequence: List[Dict]):
        """Add a turn to the current conversation"""
        if self.current_conversation_id:
            turn = ConversationTurn(
                id=str(uuid.uuid4()),
                user_input=user_input,
                robot_response=robot_response,
                timestamp=time.time(),
                context=context,
                action_sequence=action_sequence
            )

            conversation = self.conversations[self.current_conversation_id]
            conversation.append(turn)

            # Trim conversation if too long
            if len(conversation) > self.max_conversation_history:
                conversation.pop(0)

    def get_context(self) -> Dict:
        """Get current conversation context"""
        if not self.current_conversation_id:
            return {}

        conversation = self.conversations.get(self.current_conversation_id, [])
        if not conversation:
            return {}

        # Return the context from the last turn
        return conversation[-1].context

    def infer_missing_information(self, command: str) -> Dict:
        """Infer missing information from conversation context"""
        context = self.get_context()

        # Example: If user says "pick it up" but doesn't specify object,
        # infer from previous context
        if 'it' in command.lower() and context.get('last_detected_object'):
            return {'inferred_object': context['last_detected_object']}

        # Example: If user says "go there" but doesn't specify location,
        # infer from previous context
        if 'there' in command.lower() and context.get('last_mentioned_location'):
            return {'inferred_location': context['last_mentioned_location']}

        return {}

    def generate_response(self, intent: str, entities: Dict,
                         execution_result: Dict) -> str:
        """Generate appropriate robot response"""
        response_templates = {
            'navigation': {
                'success': f"I'm on my way to the {entities.get('location', 'destination')}.",
                'failure': f"I couldn't reach the {entities.get('location', 'destination')}. Is there another way?"
            },
            'manipulation': {
                'success': f"I've picked up the {entities.get('object', 'object')}.",
                'failure': f"I couldn't pick up the {entities.get('object', 'object')}. It might be too heavy or in a difficult position."
            },
            'question': {
                'answered': f"{execution_result.get('answer', 'I have provided the information you requested.')}",
                'unanswered': "I don't have enough information to answer that question. Could you provide more details?"
            },
            'greeting': {
                'greeting': "Hello! How can I assist you today?",
                'response': "Nice to meet you! I'm ready to help."
            }
        }

        success = execution_result.get('success', False)

        if intent in response_templates:
            if success:
                return response_templates[intent].get('success',
                    response_templates[intent].get('greeting', 'Okay'))
            else:
                return response_templates[intent].get('failure',
                    response_templates[intent].get('unanswered', 'I encountered an issue'))
        else:
            return "I understand. How else can I help you?"

class SpeechSynthesizer:
    """Text-to-speech system for robot responses"""
    def __init__(self):
        try:
            import pyttsx3
            self.synthesizer = pyttsx3.init()

            # Configure speech properties
            self.synthesizer.setProperty('rate', 150)  # Words per minute
            self.synthesizer.setProperty('volume', 0.8)  # Volume level

            # Get available voices
            voices = self.synthesizer.getProperty('voices')
            if voices:
                self.synthesizer.setProperty('voice', voices[0].id)  # Use first voice
        except ImportError:
            print("pyttsx3 not available, using print statements only")
            self.synthesizer = None

    def speak(self, text: str, blocking: bool = True):
        """Speak the text"""
        if self.synthesizer:
            self.synthesizer.say(text)
            if blocking:
                self.synthesizer.runAndWait()
        else:
            print(f"Robot says: {text}")

    def set_speech_properties(self, rate: int = None, volume: float = None, voice: str = None):
        """Adjust speech properties"""
        if self.synthesizer:
            if rate is not None:
                self.synthesizer.setProperty('rate', rate)
            if volume is not None:
                self.synthesizer.setProperty('volume', volume)
            if voice is not None:
                self.synthesizer.setProperty('voice', voice)
```

## Integration with Robot Control Systems

### Complete Conversational Robot System

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from builtin_interfaces.msg import Time

class ConversationalRobotNode(Node):
    """Complete conversational robot node integrating all components"""
    def __init__(self, openai_api_key: str = None):
        super().__init__('conversational_robot')

        # Initialize components
        self.speech_recognizer = AdvancedSpeechRecognizer(openai_api_key)
        self.nlu_system = NaturalLanguageUnderstanding(openai_api_key)
        self.dialog_manager = DialogManager()
        self.speech_synthesizer = SpeechSynthesizer()

        # Robot state and capabilities
        self.robot_capabilities = [
            'navigation', 'manipulation', 'perception', 'greeting',
            'object_detection', 'mapping', 'grasping', 'placement'
        ]

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speech_pub = self.create_publisher(String, '/robot_speech', 10)
        self.action_status_pub = self.create_publisher(String, '/action_status', 10)

        # Subscribers for sensor data
        self.image_sub = self.create_subscription(
            Image, '/camera/image_rect_color', self.image_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Timer for conversation loop
        self.conversation_timer = self.create_timer(1.0, self.conversation_loop)

        # Current robot state
        self.current_image = None
        self.current_imu = None
        self.robot_pose = None

        # Conversation state
        self.is_active = False
        self.last_command_time = time.time()
        self.command_timeout = 30.0  # seconds

    def image_callback(self, msg):
        """Process camera image for visual context"""
        self.current_image = msg

    def imu_callback(self, msg):
        """Process IMU data for orientation and balance"""
        self.current_imu = msg

    def conversation_loop(self):
        """Main conversation processing loop"""
        if not self.is_active:
            # Check for wake word to activate
            if self.speech_recognizer.listen_for_wake_word(timeout=1.0):
                self.activate_conversation()
        else:
            # Check if conversation should timeout
            if time.time() - self.last_command_time > self.command_timeout:
                self.deactivate_conversation()
                self.speech_synthesizer.speak("Conversation timeout. Please say wake word to continue.")
                return

            # Listen for command
            result = self.speech_recognizer.listen_for_command(timeout=5.0)
            if result:
                self.process_command(result.text)
                self.last_command_time = time.time()

    def activate_conversation(self):
        """Activate conversation mode"""
        self.is_active = True
        self.last_command_time = time.time()
        self.dialog_manager.start_conversation()

        self.speech_synthesizer.speak("Hello! I'm ready to help. What would you like me to do?")
        self.get_logger().info("Conversation activated")

    def deactivate_conversation(self):
        """Deactivate conversation mode"""
        self.is_active = False
        self.get_logger().info("Conversation deactivated")

    def process_command(self, command: str):
        """Process natural language command"""
        self.get_logger().info(f"Processing command: {command}")

        # Get contextual information
        context = self.get_environment_context()

        # Parse command
        parsed_command = self.speech_recognizer.get_contextual_command(command)

        # Understand command with NLU
        understanding_result = self.nlu_system.understand_command(
            parsed_command['parsed_command']['original_command'],
            self.robot_capabilities
        )

        if understanding_result['success']:
            # Execute action sequence
            execution_result = self.execute_action_sequence(
                understanding_result['action_sequence']
            )

            # Generate response
            response = self.dialog_manager.generate_response(
                parsed_command['action'],
                {'objects': parsed_command['objects'], 'locations': parsed_command['locations']},
                execution_result
            )

            # Speak response
            self.speech_synthesizer.speak(response)

            # Add to conversation history
            self.dialog_manager.add_turn(
                command, response, context, understanding_result['action_sequence']
            )

            # Publish robot speech
            speech_msg = String()
            speech_msg.data = response
            self.speech_pub.publish(speech_msg)

        else:
            # Failed to understand command
            self.speech_synthesizer.speak("I'm sorry, I didn't understand that command. Could you please rephrase it?")
            self.get_logger().warn(f"Failed to understand command: {command}")

    def get_environment_context(self) -> Dict:
        """Get current environment context"""
        context = {}

        # Add visual context if available
        if self.current_image:
            # In practice, this would process the image to extract context
            context['has_visual_data'] = True
            context['image_timestamp'] = self.current_image.header.stamp.sec + self.current_image.header.stamp.nanosec * 1e-9

        # Add IMU context
        if self.current_imu:
            context['orientation'] = [
                self.current_imu.orientation.x,
                self.current_imu.orientation.y,
                self.current_imu.orientation.z,
                self.current_imu.orientation.w
            ]
            context['angular_velocity'] = [
                self.current_imu.angular_velocity.x,
                self.current_imu.angular_velocity.y,
                self.current_imu.angular_velocity.z
            ]

        # Add robot state
        context['robot_pose'] = self.robot_pose
        context['robot_capabilities'] = self.robot_capabilities

        return context

    def execute_action_sequence(self, action_sequence: List[Dict]) -> Dict:
        """Execute a sequence of actions"""
        results = []
        success = True

        for i, action in enumerate(action_sequence):
            self.get_logger().info(f"Executing action {i+1}/{len(action_sequence)}: {action['action']}")

            # Execute action based on type
            result = self.execute_single_action(action)
            results.append(result)

            if not result.get('success', False):
                success = False
                self.get_logger().error(f"Action failed: {action['action']}")
                break

            # Wait between actions to allow completion
            time.sleep(0.5)

        return {
            'success': success,
            'action_results': results,
            'completed_actions': len([r for r in results if r.get('success', False)])
        }

    def execute_single_action(self, action: Dict) -> Dict:
        """Execute a single action"""
        action_type = action['action']
        parameters = action.get('parameters', {})

        if action_type == 'move_to':
            return self.execute_navigation_action(parameters)
        elif action_type == 'pick_object':
            return self.execute_manipulation_action(parameters)
        elif action_type == 'detect_objects':
            return self.execute_perception_action(parameters)
        elif action_type == 'greet_person':
            return self.execute_social_action(parameters)
        elif action_type == 'answer_question':
            return self.execute_question_answer_action(parameters)
        else:
            return {
                'success': False,
                'error': f'Unknown action type: {action_type}',
                'action_executed': action_type
            }

    def execute_navigation_action(self, parameters: Dict) -> Dict:
        """Execute navigation action"""
        try:
            target_location = parameters.get('location', 'unknown')

            # In a real system, this would use Nav2 to navigate
            # For simulation, we'll just publish a command
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.5  # Move forward at 0.5 m/s
            cmd_msg.angular.z = 0.1  # Slight turn

            self.cmd_vel_pub.publish(cmd_msg)

            self.get_logger().info(f"Moving to {target_location}")

            # Simulate navigation completion after delay
            time.sleep(2.0)

            # Stop movement
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)

            return {
                'success': True,
                'action_executed': 'move_to',
                'target_location': target_location
            }

        except Exception as e:
            return {
                'success': False,
                'error': str(e),
                'action_executed': 'move_to'
            }

    def execute_manipulation_action(self, parameters: Dict) -> Dict:
        """Execute manipulation action"""
        try:
            object_name = parameters.get('object_name', 'unknown')

            self.get_logger().info(f"Attempting to pick up {object_name}")

            # In a real system, this would control robot arms/grippers
            # For simulation, we'll just log the action
            time.sleep(3.0)  # Simulate manipulation time

            return {
                'success': True,
                'action_executed': 'pick_object',
                'object_name': object_name
            }

        except Exception as e:
            return {
                'success': False,
                'error': str(e),
                'action_executed': 'pick_object'
            }

    def execute_perception_action(self, parameters: Dict) -> Dict:
        """Execute perception action"""
        try:
            object_types = parameters.get('object_types', ['all'])

            self.get_logger().info(f"Detecting objects: {object_types}")

            # In a real system, this would process camera/sensor data
            # For simulation, return dummy detection
            detected_objects = [
                {'name': 'cup', 'confidence': 0.85, 'position': [1.0, 0.5, 0.0]},
                {'name': 'chair', 'confidence': 0.92, 'position': [2.0, -0.2, 0.0]}
            ]

            # Update context with detected objects
            context = self.dialog_manager.get_context()
            context['last_detected_objects'] = detected_objects

            return {
                'success': True,
                'action_executed': 'detect_objects',
                'detected_objects': detected_objects
            }

        except Exception as e:
            return {
                'success': False,
                'error': str(e),
                'action_executed': 'detect_objects'
            }

    def execute_social_action(self, parameters: Dict) -> Dict:
        """Execute social interaction action"""
        try:
            greeting_type = parameters.get('greeting_type', 'casual')

            if greeting_type == 'casual':
                greeting = "Hello! Nice to meet you."
            elif greeting_type == 'formal':
                greeting = "Good day. How may I assist you?"
            else:
                greeting = "Hi there!"

            self.speech_synthesizer.speak(greeting)

            return {
                'success': True,
                'action_executed': 'greet_person',
                'greeting': greeting
            }

        except Exception as e:
            return {
                'success': False,
                'error': str(e),
                'action_executed': 'greet_person'
            }

    def execute_question_answer_action(self, parameters: Dict) -> Dict:
        """Execute question answering action"""
        try:
            question = parameters.get('question', '')

            self.get_logger().info(f"Answering question: {question}")

            # In a real system, this would use an LLM to generate answers
            # For simulation, return a simple answer
            answer = f"I understand your question about '{question[:20]}...'. " \
                    "I can help with various tasks including navigation, " \
                    "object manipulation, and information retrieval."

            self.speech_synthesizer.speak(answer)

            return {
                'success': True,
                'action_executed': 'answer_question',
                'question': question,
                'answer': answer
            }

        except Exception as e:
            return {
                'success': False,
                'error': str(e),
                'action_executed': 'answer_question'
            }

def main():
    """Main function to run the conversational robot"""
    rclpy.init()

    # Get OpenAI API key from parameter or environment
    api_key = os.getenv('OPENAI_API_KEY')
    if not api_key:
        print("Warning: OPENAI_API_KEY not found in environment. Some features may be limited.")

    robot_node = ConversationalRobotNode(api_key)

    try:
        rclpy.spin(robot_node)
    except KeyboardInterrupt:
        robot_node.get_logger().info('Shutting down conversational robot')
    finally:
        robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Capstone: Autonomous Humanoid with Conversational AI

### Complete Integration Example

```python
class AutonomousHumanoidCapstone:
    """Complete capstone implementation: Autonomous Humanoid with Conversational AI"""
    def __init__(self, openai_api_key: str = None):
        # Initialize all subsystems
        self.speech_system = AdvancedSpeechRecognizer(openai_api_key)
        self.nlu_system = NaturalLanguageUnderstanding(openai_api_key)
        self.dialog_system = DialogManager()
        self.speech_synthesizer = SpeechSynthesizer()
        self.vision_system = IsaacPerceptionSystem()
        self.control_system = HumanoidController()
        self.navigation_system = HumanoidNavigationSystem()
        self.manipulation_system = HumanoidManipulationSystem()
        self.safety_system = SafetyMonitor()

        # Initialize conversation state
        self.conversation_active = False
        self.current_task = None
        self.task_progress = 0.0

    def start_capstone_demo(self):
        """Start the complete capstone demonstration"""
        print("Starting Autonomous Humanoid Capstone Demo...")
        print("Say 'Hey Robot' to activate conversation mode")

        while True:
            try:
                # Listen for wake word
                if self.speech_system.listen_for_wake_word(timeout=10.0):
                    print("Robot activated! Listening for command...")

                    # Activate conversation
                    self.activate_conversation()

                    # Process commands until conversation ends
                    while self.conversation_active:
                        command_result = self.speech_system.listen_for_command(timeout=10.0)

                        if command_result:
                            self.process_command(command_result.text)
                        else:
                            # No command received, check if conversation should end
                            time.sleep(1.0)

                            # End conversation after period of inactivity
                            # (would be implemented with a timer in a real system)

                time.sleep(0.1)  # Small delay to prevent busy waiting

            except KeyboardInterrupt:
                print("\nShutting down capstone demo...")
                break

    def activate_conversation(self):
        """Activate conversation mode"""
        self.conversation_active = True
        self.dialog_system.start_conversation()

        # Robot performs activation gesture
        self.control_system.execute_gesture('attention_pose')

        # Speak activation message
        self.speech_synthesizer.speak("Hello! I'm your humanoid assistant. How can I help you today?")

    def process_command(self, command: str):
        """Process natural language command in the capstone system"""
        print(f"Processing command: {command}")

        # Get environment context
        context = self.get_environment_context()

        # Parse command contextually
        parsed_command = self.speech_system.get_contextual_command(command)

        # Understand command with NLU
        understanding_result = self.nlu_system.understand_command(
            command, self.control_system.get_capabilities()
        )

        if understanding_result['success']:
            # Execute the action sequence
            execution_result = self.execute_capstone_task(
                understanding_result['action_sequence']
            )

            # Generate response based on execution
            response = self.dialog_system.generate_response(
                parsed_command['action'],
                parsed_command,
                execution_result
            )

            # Speak response
            self.speech_synthesizer.speak(response)

            # Log conversation turn
            self.dialog_system.add_turn(
                command, response, context, understanding_result['action_sequence']
            )

            # Check if this was the final action in a task
            if self.is_task_complete(understanding_result['action_sequence']):
                self.complete_task()
        else:
            # Handle command understanding failure
            self.speech_synthesizer.speak(
                "I'm sorry, I couldn't understand that command. Could you please rephrase it?"
            )

    def get_environment_context(self) -> Dict:
        """Get comprehensive environment context"""
        # Get visual information
        visual_context = self.vision_system.get_current_scene_analysis()

        # Get robot state
        robot_state = self.control_system.get_current_state()

        # Get navigation context
        navigation_context = self.navigation_system.get_localization_info()

        # Combine all contexts
        full_context = {
            'visual': visual_context,
            'robot_state': robot_state,
            'navigation': navigation_context,
            'timestamp': time.time()
        }

        return full_context

    def execute_capstone_task(self, action_sequence: List[Dict]) -> Dict:
        """Execute a complete capstone task with multiple action types"""
        task_results = []
        overall_success = True

        for i, action in enumerate(action_sequence):
            print(f"Executing action {i+1}/{len(action_sequence)}: {action['action']}")

            # Check safety before each action
            if not self.safety_system.check_safety():
                print("Safety check failed, stopping execution")
                return {
                    'success': False,
                    'error': 'Safety constraint violated',
                    'completed_actions': len(task_results)
                }

            # Execute action based on type
            if action['action'] == 'detect_objects':
                result = self.vision_system.detect_objects(
                    object_types=action['parameters'].get('object_types', ['all'])
                )
            elif action['action'] == 'move_to':
                result = self.navigation_system.navigate_to(
                    location=action['parameters'].get('location', 'unknown')
                )
            elif action['action'] == 'localize_robot':
                result = self.navigation_system.localize()
            elif action['action'] == 'pick_object':
                result = self.manipulation_system.pick_object(
                    object_name=action['parameters'].get('object_name', 'unknown')
                )
            elif action['action'] == 'place_object':
                result = self.manipulation_system.place_object(
                    object_name=action['parameters'].get('object_name', 'unknown'),
                    location=action['parameters'].get('location', 'default')
                )
            elif action['action'] == 'greet_person':
                result = self.control_system.execute_social_behavior(
                    behavior_type='greeting',
                    parameters=action['parameters']
                )
            elif action['action'] == 'answer_question':
                result = self.answer_question(
                    question=action['parameters'].get('question', ''),
                    context=action['parameters'].get('context', {})
                )
            else:
                result = {
                    'success': False,
                    'error': f'Unknown action: {action["action"]}',
                    'action_executed': action['action']
                }

            task_results.append(result)

            if not result.get('success', False):
                overall_success = False
                print(f"Action failed: {result.get('error', 'Unknown error')}")
                break

        return {
            'success': overall_success,
            'task_results': task_results,
            'completed_actions': len([r for r in task_results if r.get('success', False)]),
            'total_actions': len(action_sequence)
        }

    def is_task_complete(self, action_sequence: List[Dict]) -> bool:
        """Check if the current task is complete"""
        # In a real system, this would check task-specific completion criteria
        # For now, we'll just return True to indicate task completion
        return True

    def complete_task(self):
        """Handle task completion"""
        if self.current_task:
            print(f"Task completed: {self.current_task}")
            self.current_task = None
            self.task_progress = 0.0

    def answer_question(self, question: str, context: Dict) -> Dict:
        """Answer questions using integrated knowledge"""
        try:
            # In a real system, this would use the robot's knowledge base
            # combined with perception data to answer questions
            # For demonstration, we'll return a placeholder answer

            answer = f"I can answer questions about my environment and capabilities. " \
                    f"You asked: '{question}'. Based on my current understanding, " \
                    "I can provide information about objects I've detected and " \
                    "tasks I can perform."

            self.speech_synthesizer.speak(answer)

            return {
                'success': True,
                'question': question,
                'answer': answer,
                'confidence': 0.8
            }
        except Exception as e:
            return {
                'success': False,
                'error': str(e),
                'question': question
            }

class HumanoidController:
    """Humanoid robot controller for the capstone system"""
    def __init__(self):
        # Initialize humanoid-specific controllers
        self.balance_controller = HumanoidBalanceController()
        self.walking_controller = HumanoidWalkingController()
        self.arm_controller = HumanoidArmController()
        self.head_controller = HumanoidHeadController()

    def get_capabilities(self) -> List[str]:
        """Get robot capabilities for NLU system"""
        return [
            'navigation', 'manipulation', 'balance', 'greeting',
            'object_detection', 'grasping', 'placement', 'social_interaction'
        ]

    def execute_gesture(self, gesture_type: str):
        """Execute a specific gesture"""
        if gesture_type == 'attention_pose':
            # Move to attention pose
            self.head_controller.look_forward()
            self.arm_controller.move_to_attention_pose()
        elif gesture_type == 'greeting_wave':
            # Wave gesture
            self.arm_controller.wave_greeting()

    def get_current_state(self) -> Dict:
        """Get current robot state"""
        return {
            'position': self.walking_controller.get_position(),
            'orientation': self.balance_controller.get_orientation(),
            'balance_state': self.balance_controller.get_balance_metrics(),
            'arm_positions': self.arm_controller.get_joint_positions(),
            'head_position': self.head_controller.get_position()
        }

    def execute_social_behavior(self, behavior_type: str, parameters: Dict):
        """Execute social interaction behaviors"""
        if behavior_type == 'greeting':
            # Execute greeting behavior
            self.head_controller.nod()
            self.arm_controller.wave()
            return {'success': True, 'behavior_executed': 'greeting'}

        return {'success': False, 'error': 'Unknown behavior type'}

class HumanoidNavigationSystem:
    """Navigation system for humanoid robots"""
    def __init__(self):
        # Initialize navigation components
        self.localization = HumanoidLocalization()
        self.path_planner = HumanoidPathPlanner()
        self.movement_controller = HumanoidMovementController()

    def navigate_to(self, location: str) -> Dict:
        """Navigate to specified location"""
        try:
            # Plan path to location
            path = self.path_planner.plan_to_location(location)

            if not path:
                return {'success': False, 'error': f'Could not plan path to {location}'}

            # Execute navigation
            success = self.movement_controller.follow_path(path)

            return {
                'success': success,
                'destination': location,
                'path_length': len(path) if path else 0
            }
        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }

    def localize(self) -> Dict:
        """Get current robot localization"""
        position, orientation = self.localization.get_pose()
        return {
            'success': True,
            'position': position,
            'orientation': orientation,
            'confidence': 0.9
        }

    def get_localization_info(self) -> Dict:
        """Get localization information for context"""
        return {
            'position': self.localization.get_position(),
            'orientation': self.localization.get_orientation(),
            'map_frame': 'map',
            'odom_frame': 'odom'
        }

class HumanoidManipulationSystem:
    """Manipulation system for humanoid robots"""
    def __init__(self):
        self.arm_controller = HumanoidArmController()
        self.gripper_controller = HumanoidGripperController()
        self.object_detector = IsaacPerceptionSystem()

    def pick_object(self, object_name: str) -> Dict:
        """Pick up an object"""
        try:
            # Detect object
            detected_objects = self.object_detector.detect_objects([object_name])

            if not detected_objects:
                return {'success': False, 'error': f'Could not find {object_name}'}

            # Get object position
            target_object = detected_objects[0]  # Take first detected instance
            object_position = target_object['position']

            # Approach object
            approach_success = self.arm_controller.approach_object(object_position)
            if not approach_success:
                return {'success': False, 'error': 'Could not approach object'}

            # Grasp object
            grasp_success = self.gripper_controller.grasp_object()
            if not grasp_success:
                return {'success': False, 'error': 'Could not grasp object'}

            return {
                'success': True,
                'object_name': object_name,
                'object_position': object_position
            }
        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }

    def place_object(self, object_name: str, location: str) -> Dict:
        """Place object at location"""
        try:
            # Get location coordinates
            location_coords = self.get_location_coordinates(location)

            # Move to location
            move_success = self.arm_controller.move_to_position(location_coords)
            if not move_success:
                return {'success': False, 'error': 'Could not move to location'}

            # Release object
            release_success = self.gripper_controller.release_object()
            if not release_success:
                return {'success': False, 'error': 'Could not release object'}

            return {
                'success': True,
                'object_name': object_name,
                'location': location,
                'placed_position': location_coords
            }
        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }

    def get_location_coordinates(self, location: str) -> List[float]:
        """Get coordinates for named location"""
        # In a real system, this would use a map of known locations
        # For demonstration, return placeholder coordinates
        location_map = {
            'table': [1.0, 0.0, 0.0],
            'counter': [1.5, 0.5, 0.0],
            'shelf': [0.5, -1.0, 0.0],
            'floor': [0.0, 0.0, -0.5]  # Below surface level
        }
        return location_map.get(location, [0.0, 0.0, 0.0])
```

## Deployment and Evaluation

### Performance Evaluation Metrics

```python
import time
from typing import Dict, List
import numpy as np

class CapstoneEvaluator:
    """Evaluate the performance of the conversational humanoid system"""
    def __init__(self):
        self.metrics = {
            'response_accuracy': [],
            'execution_success_rate': [],
            'interaction_latency': [],
            'safety_violations': [],
            'user_satisfaction': []
        }

    def evaluate_response_accuracy(self, predicted_action: str,
                                 expected_action: str) -> float:
        """Evaluate accuracy of response prediction"""
        if predicted_action == expected_action:
            return 1.0
        elif self.actions_are_semantically_similar(predicted_action, expected_action):
            return 0.8
        else:
            return 0.0

    def actions_are_semantically_similar(self, action1: str, action2: str) -> bool:
        """Check if two actions are semantically similar"""
        # Define action similarity mappings
        similar_actions = {
            'move_to': ['navigate_to', 'go_to', 'travel_to'],
            'pick_object': ['grasp_object', 'take_object', 'lift_object'],
            'place_object': ['set_object', 'put_down', 'release_object'],
            'detect_objects': ['find_objects', 'locate_objects', 'scan_environment']
        }

        for base_action, similar_list in similar_actions.items():
            if action1 == base_action and action2 in similar_list:
                return True
            if action2 == base_action and action1 in similar_list:
                return True

        return action1 == action2

    def evaluate_execution_success(self, action_sequence: List[Dict]) -> float:
        """Evaluate success rate of action execution"""
        successful_actions = sum(1 for action in action_sequence
                               if action.get('success', False))
        total_actions = len(action_sequence)

        if total_actions == 0:
            return 0.0

        return successful_actions / total_actions

    def measure_interaction_latency(self, start_time: float,
                                  end_time: float) -> float:
        """Measure latency of interaction"""
        latency = end_time - start_time
        self.metrics['interaction_latency'].append(latency)
        return latency

    def check_safety_compliance(self, safety_monitor: SafetyMonitor) -> Dict:
        """Check safety compliance during interaction"""
        safety_status = safety_monitor.get_safety_status()

        safety_report = {
            'is_safe': safety_status['safe'],
            'violations_count': safety_status['violations'],
            'emergency_stops': safety_status.get('emergency_stops', 0)
        }

        self.metrics['safety_violations'].append(safety_report['violations_count'])

        return safety_report

    def evaluate_user_satisfaction(self, user_feedback: str) -> float:
        """Evaluate user satisfaction from feedback"""
        # Simple sentiment analysis for user feedback
        positive_keywords = ['good', 'great', 'excellent', 'helpful', 'perfect', 'nice']
        negative_keywords = ['bad', 'poor', 'terrible', 'frustrating', 'wrong', 'disappointing']

        feedback_lower = user_feedback.lower()

        positive_count = sum(1 for word in positive_keywords if word in feedback_lower)
        negative_count = sum(1 for word in negative_keywords if word in feedback_lower)

        if positive_count + negative_count == 0:
            return 0.5  # Neutral if no sentiment words found

        satisfaction_score = positive_count / (positive_count + negative_count)
        self.metrics['user_satisfaction'].append(satisfaction_score)

        return satisfaction_score

    def get_performance_report(self) -> Dict:
        """Generate performance evaluation report"""
        report = {}

        if self.metrics['response_accuracy']:
            report['avg_response_accuracy'] = np.mean(self.metrics['response_accuracy'])

        if self.metrics['execution_success_rate']:
            report['avg_execution_success_rate'] = np.mean(self.metrics['execution_success_rate'])

        if self.metrics['interaction_latency']:
            report['avg_interaction_latency'] = np.mean(self.metrics['interaction_latency'])
            report['max_interaction_latency'] = np.max(self.metrics['interaction_latency'])

        if self.metrics['safety_violations']:
            report['total_safety_violations'] = sum(self.metrics['safety_violations'])
            report['avg_safety_violations'] = np.mean(self.metrics['safety_violations'])

        if self.metrics['user_satisfaction']:
            report['avg_user_satisfaction'] = np.mean(self.metrics['user_satisfaction'])

        return report

def run_capstone_evaluation():
    """Run complete evaluation of the capstone system"""
    evaluator = CapstoneEvaluator()
    capstone_system = AutonomousHumanoidCapstone()

    print("Starting capstone system evaluation...")

    # Define test scenarios
    test_scenarios = [
        {
            'command': 'Please go to the kitchen and bring me a cup',
            'expected_actions': ['navigate_to', 'detect_objects', 'pick_object', 'navigate_to'],
            'location': 'kitchen'
        },
        {
            'command': 'Find the red ball and put it on the table',
            'expected_actions': ['detect_objects', 'pick_object', 'navigate_to', 'place_object'],
            'location': 'living_room'
        },
        {
            'command': 'Tell me what objects you see in the room',
            'expected_actions': ['detect_objects', 'answer_question'],
            'location': 'office'
        }
    ]

    results = []

    for scenario in test_scenarios:
        print(f"\nRunning scenario: {scenario['command']}")

        start_time = time.time()

        # Execute scenario
        understanding_result = capstone_system.nlu_system.understand_command(
            scenario['command'], capstone_system.control_system.get_capabilities()
        )

        if understanding_result['success']:
            execution_result = capstone_system.execute_capstone_task(
                understanding_result['action_sequence']
            )

            end_time = time.time()
            latency = evaluator.measure_interaction_latency(start_time, end_time)

            # Evaluate accuracy
            predicted_actions = [action['action'] for action in understanding_result['action_sequence']]
            accuracy = evaluator.evaluate_response_accuracy(
                str(predicted_actions), str(scenario['expected_actions'])
            )

            # Evaluate execution success
            success_rate = evaluator.evaluate_execution_success(execution_result['task_results'])

            # Check safety
            safety_report = evaluator.check_safety_compliance(capstone_system.safety_system)

            scenario_result = {
                'command': scenario['command'],
                'predicted_actions': predicted_actions,
                'expected_actions': scenario['expected_actions'],
                'accuracy': accuracy,
                'success_rate': success_rate,
                'latency': latency,
                'safety': safety_report,
                'passed': accuracy >= 0.7 and success_rate >= 0.8 and safety_report['is_safe']
            }

            results.append(scenario_result)
            print(f"Scenario result: {'PASSED' if scenario_result['passed'] else 'FAILED'}")
        else:
            print("Failed to understand command")
            results.append({
                'command': scenario['command'],
                'error': 'Command understanding failed',
                'passed': False
            })

    # Generate final report
    final_report = evaluator.get_performance_report()
    final_report['test_scenarios'] = len(test_scenarios)
    final_report['successful_scenarios'] = sum(1 for r in results if r.get('passed', False))
    final_report['scenario_success_rate'] = final_report['successful_scenarios'] / len(test_scenarios)

    print(f"\nFinal Evaluation Report:")
    print(f"- Scenario Success Rate: {final_report['scenario_success_rate']:.2f}")
    print(f"- Average Response Accuracy: {final_report.get('avg_response_accuracy', 0):.2f}")
    print(f"- Average Execution Success Rate: {final_report.get('avg_execution_success_rate', 0):.2f}")
    print(f"- Average Interaction Latency: {final_report.get('avg_interaction_latency', 0):.2f}s")
    print(f"- Total Safety Violations: {final_report.get('total_safety_violations', 0)}")

    return final_report
```

## Hands-on Exercise

Complete the capstone project implementation:

1. Integrate all components: speech recognition, NLU, dialog management, and robot control
2. Implement the complete conversational robot system
3. Test with various natural language commands
4. Evaluate system performance using the provided metrics
5. Optimize for real-time operation
6. Demonstrate the complete autonomous humanoid system

## Review Questions

1. How do Vision-Language-Action systems enable more natural human-robot interaction?
2. What are the key challenges in integrating conversational AI with physical robots?
3. How does multimodal perception improve robot understanding of commands?
4. What safety considerations are essential for conversational humanoid robots?

## Further Reading

- "Conversational Robotics" by Bilge Mutlu and Jodi Forlizzi
- "Human-Robot Interaction: An Introduction" by Kerstin Dautenhahn
- "Spoken Dialogue Systems" by Julia Hirschberg and Amanda Stent
- NVIDIA Isaac ROS Documentation for Conversational AI