---
sidebar_position: 1
title: "Chapter 7: API Integration - Conversational Robotics"
---

# Chapter 7: API Integration - Conversational Robotics

## Learning Objectives

By the end of this chapter, you should be able to:

**Remember**: List the components of conversational AI systems and their functions in robotics

**Understand**: Explain how dialogue systems enable natural human-robot interaction

**Apply**: Implement a conversational interface for a robot that processes speech and text

**Analyze**: Evaluate the effectiveness of different dialogue strategies in robotics

**Evaluate**: Assess the impact of conversational robotics on user experience and engagement

**Create**: Design a complete conversational robotics system integrating speech, vision, and action

## 7.1 Conversational AI Fundamentals

Conversational AI in robotics represents the integration of natural language processing, speech recognition, and dialogue management systems to enable human-like interaction with robots. Unlike traditional command-based interfaces, conversational robots can engage in multi-turn dialogues, understand context, and respond appropriately to natural language input.

### Core Components of Conversational Robotics

The architecture of conversational robots typically includes several key components:

- **Automatic Speech Recognition (ASR)**: Converts spoken language to text
- **Natural Language Understanding (NLU)**: Interprets the meaning of text input
- **Dialogue Manager**: Maintains conversation state and manages turn-taking
- **Natural Language Generation (NLG)**: Creates natural language responses
- **Text-to-Speech (TTS)**: Converts text responses to spoken output
- **Multimodal Integration**: Combines speech with visual and other sensory information

```python
import asyncio
import numpy as np
import speech_recognition as sr
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from enum import Enum

class DialogueState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    SPEAKING = "speaking"
    WAITING_FOR_CONFIRMATION = "waiting_confirmation"

@dataclass
class ConversationContext:
    """Context information for maintaining conversation state"""
    history: List[Dict[str, str]]
    current_intent: str
    entities: Dict[str, Any]
    user_profile: Dict[str, Any]
    session_id: str
    timestamp: float

class SpeechRecognitionSystem:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
        
        # Set up for specific speech recognition
        self.recognizer.energy_threshold = 400
        self.recognizer.dynamic_energy_threshold = True
        
    def listen_for_speech(self, timeout: float = 5.0) -> Optional[str]:
        """
        Listen for speech input and convert to text
        """
        try:
            with self.microphone as source:
                print("Listening...")
                audio = self.recognizer.listen(source, timeout=timeout)
            
            # Use Google's speech recognition service
            text = self.recognizer.recognize_google(audio)
            print(f"Recognized: {text}")
            return text
            
        except sr.WaitTimeoutError:
            print("Timeout: No speech detected")
            return None
        except sr.UnknownValueError:
            print("Could not understand audio")
            return None
        except sr.RequestError as e:
            print(f"Error with speech recognition service: {e}")
            return None
    
    def continuous_listening(self, callback):
        """
        Set up continuous listening with callback for recognized speech
        """
        def audio_callback(recognizer, audio):
            try:
                text = recognizer.recognize_google(audio)
                callback(text)
            except sr.UnknownValueError:
                pass  # Ignore unrecognized audio
            except sr.RequestError:
                pass  # Ignore service errors
        
        self.recognizer.listen_in_background(self.microphone, audio_callback)

class NaturalLanguageUnderstanding:
    def __init__(self):
        # Intent classification model (simplified for this example)
        self.intent_keywords = {
            "greeting": ["hello", "hi", "hey", "good morning", "good evening"],
            "navigation": ["go to", "move to", "walk to", "navigate to", "take me to"],
            "object_interaction": ["pick", "grasp", "get", "bring", "fetch", "put", "place"],
            "information_request": ["what", "where", "when", "who", "how", "tell me"],
            "confirmation": ["yes", "no", "ok", "okay", "sure", "cancel"],
            "stop": ["stop", "halt", "pause", "quit", "exit"]
        }
        
        # Entity extraction patterns
        self.location_entities = {
            "kitchen": ["kitchen", "dining area", "cooking area"],
            "living_room": ["living room", "sitting room", "lounge", "family room"],
            "bedroom": ["bedroom", "sleeping area", "bed area"],
            "office": ["office", "study", "workspace", "desk area"],
            "entrance": ["entrance", "front door", "entry", "living area"]
        }
        
    def extract_intent_and_entities(self, text: str) -> Dict[str, Any]:
        """
        Extract intent and entities from input text
        """
        text_lower = text.lower()
        intent = None
        entities = {}
        
        # Extract intent
        for intent_name, keywords in self.intent_keywords.items():
            if any(keyword in text_lower for keyword in keywords):
                intent = intent_name
                break
        
        # Extract location entities
        for location, aliases in self.location_entities.items():
            if any(alias in text_lower for alias in aliases):
                entities['location'] = location
                break
        
        # Extract other entities based on context
        if intent == "object_interaction":
            # Simple object extraction (in practice, this would be more sophisticated)
            words = text_lower.split()
            for i, word in enumerate(words):
                if word in ["the", "a", "an", "some"]:
                    if i + 1 < len(words):
                        entities['object'] = words[i + 1]
                        break
        
        return {
            "intent": intent,
            "entities": entities,
            "confidence": 0.8  # Simplified confidence
        }
```

### Dialogue Management

Effective dialogue management is crucial for creating natural, engaging conversations:

```python
class DialogueManager:
    def __init__(self):
        self.current_state = DialogueState.IDLE
        self.conversation_context = ConversationContext(
            history=[],
            current_intent="",
            entities={},
            user_profile={},
            session_id="",
            timestamp=0
        )
        self.context_memory = []  # Recent conversation history
        
    def process_input(self, text: str) -> Dict[str, Any]:
        """
        Process user input and determine appropriate response
        """
        nlu = NaturalLanguageUnderstanding()
        understanding = nlu.extract_intent_and_entities(text)
        
        intent = understanding["intent"]
        entities = understanding["entities"]
        confidence = understanding["confidence"]
        
        # Update conversation context
        self.conversation_context.history.append({
            "user": text,
            "intent": intent,
            "entities": entities,
            "timestamp": self.conversation_context.timestamp
        })
        
        # Update context with new entities
        self.conversation_context.entities.update(entities)
        
        # Determine response based on intent and context
        response = self.generate_response(intent, entities, confidence)
        
        return {
            "response": response,
            "intent": intent,
            "entities": entities,
            "confidence": confidence
        }
    
    def generate_response(self, intent: str, entities: Dict, confidence: float) -> str:
        """
        Generate appropriate response based on intent and context
        """
        if confidence < 0.5:
            return "I'm sorry, I didn't quite understand that. Could you please repeat?"
        
        if intent == "greeting":
            return self.handle_greeting(entities)
        elif intent == "navigation":
            return self.handle_navigation(entities)
        elif intent == "object_interaction":
            return self.handle_object_interaction(entities)
        elif intent == "information_request":
            return self.handle_information_request(entities)
        elif intent == "confirmation":
            return self.handle_confirmation(entities)
        elif intent == "stop":
            return self.handle_stop(entities)
        else:
            # Default response
            if entities:
                return f"I understand you want to do something with the {list(entities.keys())[0]}. Could you please be more specific?"
            else:
                return "I'm here to help. How can I assist you today?"
    
    def handle_greeting(self, entities: Dict) -> str:
        """Handle greeting intents"""
        import random
        greetings = [
            "Hello! How can I help you today?",
            "Hi there! What can I do for you?",
            "Good to see you! How are you doing?",
            "Hello! I'm ready to assist you."
        ]
        return random.choice(greetings)
    
    def handle_navigation(self, entities: Dict) -> str:
        """Handle navigation commands"""
        if 'location' in entities:
            location = entities['location']
            return f"Okay, I'll navigate to the {location}. Please follow me."
        else:
            return "I can help you navigate, but I need to know where you'd like to go."
    
    def handle_object_interaction(self, entities: Dict) -> str:
        """Handle object interaction commands"""
        if 'object' in entities:
            obj = entities['object']
            return f"I'll help you with the {obj}. Can you show me where it is?"
        else:
            return "I can help you interact with objects. What would you like me to help you with?"
    
    def handle_information_request(self, entities: Dict) -> str:
        """Handle information requests"""
        return "I can provide information if you specify what you'd like to know."
    
    def handle_confirmation(self, entities: Dict) -> str:
        """Handle confirmation responses"""
        return "Got it. What else can I help you with?"
    
    def handle_stop(self, entities: Dict) -> str:
        """Handle stop commands"""
        return "Okay, stopping current operation. How else can I assist you?"
```

## 7.2 Speech Recognition and Synthesis

### Automatic Speech Recognition (ASR)

Advanced speech recognition systems are crucial for conversational robots, allowing them to understand human speech in various conditions:

```python
class AdvancedSpeechRecognition:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphones = sr.Microphone.list_microphone_names()
        
        # Configuration for robotics environment
        self.config = {
            'energy_threshold': 400,
            'dynamic_energy_threshold': True,
            'pause_threshold': 0.8,
            'phrase_threshold': 0.3,
            'non_speaking_duration': 0.5
        }
        
        # Initialize multiple microphones for better hearing
        self.audio_sources = []
        for i, mic_name in enumerate(self.microphones):
            if 'array' in mic_name.lower() or 'beam' in mic_name.lower():
                self.audio_sources.append(sr.Microphone(device_index=i))
        
        # If no special microphones found, use default
        if not self.audio_sources:
            self.audio_sources.append(sr.Microphone())
        
        # Configure recognizer
        self.recognizer.energy_threshold = self.config['energy_threshold']
        self.recognizer.pause_threshold = self.config['pause_threshold']
        self.recognizer.phrase_threshold = self.config['phrase_threshold']
        self.recognizer.non_speaking_duration = self.config['non_speaking_duration']
    
    def adaptive_noise_cancellation(self, source):
        """
        Adapt to changing noise conditions
        """
        with source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
    
    def keyword_spotting(self, keywords: List[str], callback_func, timeout: int = 10):
        """
        Listen for specific keywords and call back when detected
        """
        import time
        
        keyword_lower = [kw.lower() for kw in keywords]
        
        def listen_loop():
            for source in self.audio_sources:
                with source:
                    try:
                        audio = self.recognizer.listen(source, timeout=timeout)
                        text = self.recognizer.recognize_google(audio).lower()
                        
                        for keyword in keyword_lower:
                            if keyword in text:
                                # Call the callback function with the detected keyword
                                callback_func(keyword, text)
                                return True
                    except sr.WaitTimeoutError:
                        continue
                    except sr.UnknownValueError:
                        continue
                    except sr.RequestError:
                        continue
            return False
        
        # Run the listening loop
        return listen_loop()
    
    def multi_channel_processing(self):
        """
        Process audio from multiple channels for better recognition
        """
        # In a real implementation, this would handle multiple audio streams
        # for spatial audio processing and noise cancellation
        pass

class SpeechSynthesisSystem:
    def __init__(self):
        # For this example, we'll use pyttsx3 for local synthesis
        # In practice, you might use cloud services like AWS Polly or Google TTS
        try:
            import pyttsx3
            self.engine = pyttsx3.init()
            
            # Configure speech properties
            self.engine.setProperty('rate', 150)  # Speed of speech
            self.engine.setProperty('volume', 0.9)  # Volume level (0.0 to 1.0)
            
            # Get available voices and select one that sounds natural
            voices = self.engine.getProperty('voices')
            if voices:
                # Select a natural-sounding voice (usually the first female voice or a specific one)
                for voice in voices:
                    if 'samantha' in voice.name.lower() or 'zira' in voice.name.lower() or 'jane' in voice.name.lower():
                        self.engine.setProperty('voice', voice.id)
                        break
                    elif 'english' in voice.name.lower() or 'en' in voice.name.lower():
                        self.engine.setProperty('voice', voice.id)
                        break
        except ImportError:
            print("pyttsx3 not available, using simple print method")
            self.engine = None
    
    def speak_text(self, text: str, blocking: bool = True):
        """
        Convert text to speech and play it
        """
        if self.engine:
            self.engine.say(text)
            if blocking:
                self.engine.runAndWait()
            else:
                # Non-blocking speech
                import threading
                def run_speech():
                    self.engine.runAndWait()
                
                thread = threading.Thread(target=run_speech)
                thread.start()
        else:
            # Fallback to simple print
            print(f"Robot says: {text}")
    
    def set_speech_properties(self, rate: int = None, volume: float = None, voice: str = None):
        """
        Adjust speech synthesis properties
        """
        if self.engine:
            if rate is not None:
                self.engine.setProperty('rate', rate)
            if volume is not None:
                self.engine.setProperty('volume', volume)
            if voice is not None:
                self.engine.setProperty('voice', voice)
    
    def preload_voices(self) -> List[str]:
        """
        Get list of available voices
        """
        if self.engine:
            voices = self.engine.getProperty('voices')
            return [voice.name for voice in voices]
        return []
```

### Dialogue State Tracking

Maintaining context across multiple conversational turns is essential for natural interaction:

```python
from collections import deque
import json
from datetime import datetime

class DialogueStateTracker:
    def __init__(self, max_history: int = 10):
        self.max_history = max_history
        self.conversation_history = deque(maxlen=max_history)
        self.current_topic = None
        self.user_context = {
            'preferences': {},
            'conversation_style': 'formal',  # or 'casual'
            'attention_level': 'high'  # 'high', 'medium', 'low'
        }
        self.robot_state = {
            'battery_level': 100.0,
            'current_task': None,
            'location': 'unknown',
            'capabilities': []
        }
        
    def update_context(self, user_input: str, robot_response: str, intent: str, entities: Dict):
        """
        Update the dialogue context with new information
        """
        timestamp = datetime.now().isoformat()
        
        # Create conversation entry
        entry = {
            'timestamp': timestamp,
            'user_input': user_input,
            'robot_response': robot_response,
            'intent': intent,
            'entities': entities,
            'context': self.user_context.copy(),
            'robot_state': self.robot_state.copy()
        }
        
        self.conversation_history.append(entry)
        
        # Update current topic based on intent
        if intent in ['navigation', 'object_interaction', 'information_request']:
            self.current_topic = intent
        
        # Update user context based on interaction
        self._update_user_preferences(user_input, intent)
        self._update_attention_level()
    
    def _update_user_preferences(self, user_input: str, intent: str):
        """
        Update user preferences based on interaction patterns
        """
        # Simple learning of user preferences
        if intent == "navigation":
            # Remember preferred locations
            pass
        elif intent == "object_interaction":
            # Remember object preferences
            pass
    
    def _update_attention_level(self):
        """
        Update attention level based on conversation dynamics
        """
        # This would use timing, engagement indicators, etc.
        # For now, we'll keep it simple
        pass
    
    def get_relevant_context(self, current_intent: str) -> Dict:
        """
        Get context relevant to current intent
        """
        relevant_context = {}
        
        # Look for recent entries with similar intent
        for entry in list(self.conversation_history)[-3:]:  # Check last 3 entries
            if entry['intent'] == current_intent:
                # Include entities from similar intents
                relevant_context.update(entry['entities'])
        
        # Add current user and robot state
        relevant_context['user_context'] = self.user_context
        relevant_context['robot_state'] = self.robot_state
        
        return relevant_context
    
    def serialize_context(self) -> str:
        """
        Serialize current context for storage or transmission
        """
        context_data = {
            'history': list(self.conversation_history),
            'current_topic': self.current_topic,
            'user_context': self.user_context,
            'robot_state': self.robot_state
        }
        return json.dumps(context_data, default=str)
    
    def deserialize_context(self, context_str: str):
        """
        Deserialize context from storage
        """
        context_data = json.loads(context_str)
        self.conversation_history = deque(context_data['history'], maxlen=self.max_history)
        self.current_topic = context_data['current_topic']
        self.user_context = context_data['user_context']
        self.robot_state = context_data['robot_state']
```

## 7.3 Dialog Management

### Intent Recognition and Slot Filling

Effective dialogue systems need to understand user intent and extract relevant information:

```python
import re
from typing import Tuple

class IntentRecognitionSystem:
    def __init__(self):
        # Define intent patterns with regex and keywords
        self.intent_patterns = {
            'navigation_to_location': {
                'patterns': [
                    r'go to (?:the )?(\w+)',
                    r'move to (?:the )?(\w+)',
                    r'walk to (?:the )?(\w+)',
                    r'navigate to (?:the )?(\w+)',
                    r'can you take me to (?:the )?(\w+)'
                ],
                'entities': ['location']
            },
            'grasp_object': {
                'patterns': [
                    r'pick up (?:the )?(\w+)',
                    r'grasp (?:the )?(\w+)',
                    r'get (?:the )?(\w+)',
                    r'bring me (?:the )?(\w+)',
                    r'fetch (?:the )?(\w+)'
                ],
                'entities': ['object']
            },
            'place_object': {
                'patterns': [
                    r'put (?:the )?(\w+) in (?:the )?(\w+)',
                    r'place (?:the )?(\w+) on (?:the )?(\w+)',
                    r'put (?:the )?(\w+) (?:in|on|at) (?:the )?(\w+)'
                ],
                'entities': ['object', 'location']
            },
            'greeting': {
                'patterns': [
                    r'hello',
                    r'hi',
                    r'hey',
                    r'good morning',
                    r'good evening',
                    r'good afternoon'
                ],
                'entities': []
            },
            'time_request': {
                'patterns': [
                    r'what time is it',
                    r'what is the time',
                    r'tell me the time',
                    r'current time',
                    r'clock'
                ],
                'entities': []
            }
        }
        
    def recognize_intent(self, text: str) -> Tuple[str, Dict[str, str]]:
        """
        Recognize intent and extract entities from text
        """
        text_lower = text.lower().strip()
        
        for intent_name, intent_data in self.intent_patterns.items():
            for pattern in intent_data['patterns']:
                match = re.search(pattern, text_lower)
                if match:
                    entities = {}
                    groups = match.groups()
                    
                    # Extract entities based on pattern groups
                    for i, entity_type in enumerate(intent_data['entities']):
                        if i < len(groups):
                            entities[entity_type] = groups[i]
                    
                    return intent_name, entities
        
        # If no pattern matches, use keyword-based recognition
        return self._keyword_based_recognition(text_lower)
    
    def _keyword_based_recognition(self, text_lower: str) -> Tuple[str, Dict[str, str]]:
        """
        Fallback to keyword-based intent recognition
        """
        # Simple keyword matching as fallback
        if any(word in text_lower for word in ['hello', 'hi', 'hey']):
            return 'greeting', {}
        elif any(word in text_lower for word in ['time', 'clock', 'hour']):
            return 'time_request', {}
        elif any(word in text_lower for word in ['go', 'move', 'navigate', 'walk']):
            return 'navigation_to_location', {}
        elif any(word in text_lower for word in ['pick', 'grasp', 'get', 'fetch']):
            return 'grasp_object', {}
        else:
            return 'unknown', {}

class SlotFillingSystem:
    def __init__(self):
        self.required_slots = {
            'navigation_to_location': ['location'],
            'grasp_object': ['object'],
            'place_object': ['object', 'destination'],
            'order_delivery': ['item', 'destination']
        }
        
        self.filled_slots = {}
        self.current_intent = None
    
    def start_slot_filling(self, intent: str):
        """
        Start slot filling for a particular intent
        """
        self.current_intent = intent
        self.filled_slots = {}
        
        if intent in self.required_slots:
            required = self.required_slots[intent]
            for slot in required:
                self.filled_slots[slot] = None
    
    def fill_slots(self, entities: Dict[str, str]) -> Dict[str, str]:
        """
        Fill slots with provided entities
        """
        unfilled_slots = []
        
        for slot, value in entities.items():
            if slot in self.filled_slots:
                self.filled_slots[slot] = value
        
        # Check which slots are still unfilled
        for slot, value in self.filled_slots.items():
            if value is None:
                unfilled_slots.append(slot)
        
        return {
            'filled_slots': self.filled_slots.copy(),
            'unfilled_slots': unfilled_slots,
            'completed': len(unfilled_slots) == 0
        }
    
    def is_slot_filling_complete(self) -> bool:
        """
        Check if all required slots are filled
        """
        if not self.current_intent:
            return False
            
        required_slots = self.required_slots.get(self.current_intent, [])
        for slot in required_slots:
            if self.filled_slots.get(slot) is None:
                return False
        return True
    
    def get_missing_slots(self) -> List[str]:
        """
        Get a list of missing slots
        """
        if not self.current_intent:
            return []
        
        missing = []
        required_slots = self.required_slots.get(self.current_intent, [])
        for slot in required_slots:
            if self.filled_slots.get(slot) is None:
                missing.append(slot)
        return missing
```

## 7.4 Embodied Conversational Agents

### Multimodal Interaction

Conversational robots must integrate speech, vision, and action for natural interaction:

```python
import cv2
import numpy as np
from threading import Thread
from queue import Queue
import time

class MultimodalInteractionManager:
    def __init__(self):
        self.vision_system = VisionSystem()
        self.speech_system = SpeechRecognitionSystem()
        self.dialogue_manager = DialogueManager()
        self.action_executor = ActionExecutor()
        
        # Queues for asynchronous processing
        self.vision_queue = Queue(maxsize=5)
        self.speech_queue = Queue(maxsize=5)
        self.response_queue = Queue(maxsize=5)
        
        # State tracking
        self.attention_target = None
        self.gaze_direction = (0, 0)  # In terms of head angles
        self.social_signals = {}
        
        # Processing threads
        self.vision_thread = None
        self.speech_thread = None
        self.response_thread = None
        
    def start_listening(self):
        """
        Start listening for speech input
        """
        def speech_processing_loop():
            while True:
                try:
                    text = self.speech_system.listen_for_speech()
                    if text:
                        self.speech_queue.put_nowait(text)
                except Exception as e:
                    print(f"Speech processing error: {e}")
                time.sleep(0.1)
        
        self.speech_thread = Thread(target=speech_processing_loop, daemon=True)
        self.speech_thread.start()
    
    def start_vision_processing(self):
        """
        Start processing visual input
        """
        def vision_processing_loop():
            while True:
                try:
                    # Get visual data (in a real system, this would capture from cameras)
                    visual_data = self.vision_system.get_current_scene()
                    if visual_data:
                        self.vision_queue.put_nowait(visual_data)
                except Exception as e:
                    print(f"Vision processing error: {e}")
                time.sleep(0.033)  # ~30 FPS
        
        self.vision_thread = Thread(target=vision_processing_loop, daemon=True)
        self.vision_thread.start()
    
    def start_response_processing(self):
        """
        Start processing responses
        """
        def response_processing_loop():
            while True:
                try:
                    # Process speech input
                    speech_input = None
                    try:
                        speech_input = self.speech_queue.get_nowait()
                    except:
                        pass
                    
                    if speech_input:
                        # Process the speech input with context from vision
                        visual_context = None
                        try:
                            visual_context = self.vision_queue.get_nowait()
                        except:
                            pass
                        
                        response = self.process_multimodal_input(speech_input, visual_context)
                        self.response_queue.put_nowait(response)
                        
                        # Execute speech response
                        if response:
                            self.action_executor.speak(response)
                
                except Exception as e:
                    print(f"Response processing error: {e}")
                time.sleep(0.1)
        
        self.response_thread = Thread(target=response_processing_loop, daemon=True)
        self.response_thread.start()
    
    def process_multimodal_input(self, speech_input: str, visual_context: Dict = None):
        """
        Process input that combines speech and visual information
        """
        # Extract intent and entities from speech
        dialogue_result = self.dialogue_manager.process_input(speech_input)
        
        intent = dialogue_result.get('intent', 'unknown')
        entities = dialogue_result.get('entities', {})
        
        # Integrate visual context
        if visual_context and entities.get('object'):
            # Match the mentioned object with what's seen in the visual data
            matched_object = self.vision_system.find_object_by_name(
                entities['object'], 
                visual_context
            )
            if matched_object:
                entities['object_location'] = matched_object['location']
        
        # Generate response based on multimodal input
        response = self.generate_contextual_response(
            intent, 
            entities, 
            visual_context
        )
        
        # Update attention and gaze based on interaction
        self.update_attention(speech_input, visual_context)
        
        return response
    
    def generate_contextual_response(self, intent: str, entities: Dict, visual_context: Dict = None):
        """
        Generate response considering both speech and visual context
        """
        if intent == "greeting":
            return self._generate_greeting_response(visual_context)
        elif intent == "object_interaction":
            if entities.get('object'):
                return self._generate_object_interaction_response(entities, visual_context)
        elif intent == "navigation":
            return self._generate_navigation_response(entities, visual_context)
        else:
            return "I understand you're trying to communicate with me. How can I help you?"
    
    def _generate_greeting_response(self, visual_context: Dict = None):
        """
        Generate greeting response with visual context
        """
        if visual_context:
            # Check if person is recognized
            person_detected = visual_context.get('person_detected', False)
            if person_detected:
                return "Hello! It's good to see you again."
            else:
                return "Hello! I'm here to help you. What can I do for you?"
        else:
            return "Hello! I'm here to help you. What can I do for you?"
    
    def _generate_object_interaction_response(self, entities: Dict, visual_context: Dict = None):
        """
        Generate response for object interaction with visual context
        """
        obj_name = entities.get('object', 'object')
        
        if visual_context:
            # Check if object is visible
            obj_found = self.vision_system.find_object_by_name(obj_name, visual_context)
            if obj_found:
                return f"I can see the {obj_name}. Where would you like me to {entities.get('action', 'get')} it?"
            else:
                return f"I don't see a {obj_name} nearby. Could you point it out or tell me where it is?"
        else:
            return f"I can help you with the {obj_name}. Can you show me where it is?"
    
    def _generate_navigation_response(self, entities: Dict, visual_context: Dict = None):
        """
        Generate navigation response with visual context
        """
        location = entities.get('location', 'destination')
        
        # Check if location is accessible
        if self.action_executor.is_location_reachable(location):
            return f"Okay, I'll navigate to the {location}. Please follow me."
        else:
            return f"I can help you get to the {location}, but I need more specific directions."
    
    def update_attention(self, speech_input: str, visual_context: Dict = None):
        """
        Update robot's attention based on multimodal input
        """
        # Update gaze based on who is speaking
        if visual_context and visual_context.get('person_detected'):
            # Calculate gaze direction toward the person
            person_location = visual_context.get('person_location', (0, 0))
            self.gaze_direction = self.calculate_gaze_direction(person_location)
        
        # Track attention target
        if "you" in speech_input.lower():
            self.attention_target = "user"
        elif visual_context:
            # Update attention based on visual focus
            self.attention_target = "environment"
    
    def calculate_gaze_direction(self, target_location: Tuple[float, float]):
        """
        Calculate head angles to look at a target location
        """
        # Simplified calculation - in reality, this would involve
        # inverse kinematics for the neck/head joints
        x, y = target_location
        # Convert to head angles (simplified)
        head_yaw = np.arctan2(y, x)  # Yaw angle
        head_pitch = 0.0  # Keep pitch level for now
        
        return (head_yaw, head_pitch)

class VisionSystem:
    def __init__(self):
        # Initialize camera and computer vision components
        self.camera = None
        self.object_detector = None  # Would be a trained model in practice
        self.face_recognizer = None  # Would be trained for user recognition
        
    def get_current_scene(self) -> Dict:
        """
        Get current visual information from the environment
        """
        # In a real implementation, this would capture from camera
        # and process with computer vision algorithms
        
        # For simulation, return mock data
        return {
            'objects': [
                {'name': 'mug', 'location': (1.2, 0.5, 0.8), 'confidence': 0.9},
                {'name': 'book', 'location': (1.5, 0.2, 0.8), 'confidence': 0.85}
            ],
            'person_detected': True,
            'person_location': (0.8, 0, 1.0),  # x, y, z coordinates relative to robot
            'timestamp': time.time()
        }
    
    def find_object_by_name(self, obj_name: str, scene_data: Dict) -> Dict:
        """
        Find an object by name in the current scene
        """
        for obj in scene_data.get('objects', []):
            if obj_name.lower() in obj['name'].lower() or obj_name.lower() == obj['name'].lower():
                return obj
        return None
    
    def get_people_info(self, scene_data: Dict) -> List[Dict]:
        """
        Get information about people in the scene
        """
        # Would implement face recognition and tracking
        return scene_data.get('people', [])

class ActionExecutor:
    def __init__(self):
        self.speech_synthesis = SpeechSynthesisSystem()
        self.robot_controls = None  # Would connect to actual robot
        self.animation_player = None  # Would handle robot animations
        
    def speak(self, text: str):
        """
        Make the robot speak the given text
        """
        self.speech_synthesis.speak_text(text)
    
    def move_to_location(self, location: str):
        """
        Navigate the robot to a specified location
        """
        # In a real implementation, this would:
        # - Plan a path to the location
        # - Execute navigation commands
        # - Monitor progress
        print(f"Moving to {location}")
    
    def grasp_object(self, obj_name: str):
        """
        Execute grasping action for a named object
        """
        # In a real implementation, this would:
        # - Locate the object using vision
        # - Plan grasp trajectory
        # - Execute grasp with manipulator
        print(f"Grasping {obj_name}")
    
    def is_location_reachable(self, location: str) -> bool:
        """
        Check if a location is reachable by the robot
        """
        # Check against map or navigation system
        reachable_locations = ['kitchen', 'living_room', 'office', 'bedroom', 'entrance']
        return location.lower() in [loc.lower() for loc in reachable_locations]
```

### Social Robotics Principles

Creating robots that can interact naturally with humans involves understanding social cues and responses:

```python
class SocialRoboticsEngine:
    def __init__(self):
        self.social_rules = {
            'personal_space': 1.0,  # meters
            'greeting_protocols': {
                'time_based': True,
                'frequency': 'first_meeting_only',
                'duration': 3  # seconds
            },
            'attention_management': {
                'focus_time': 5,  # seconds before shifting attention
                'polite_attention_shifting': True
            }
        }
        
        self.user_models = {}  # Store models of different users
        self.social_context = {
            'cultural_background': 'universal',
            'age_group': 'adult',
            'formality_level': 'medium'
        }
        
    def handle_greeting(self, user_id: str, context: Dict = None):
        """
        Handle greeting with appropriate social awareness
        """
        # Check if this is a returning user
        is_known_user = user_id in self.user_models
        time_of_day = context.get('time_of_day', 'day')
        
        greeting = self._select_greeting(is_known_user, time_of_day)
        
        # Add personalization if available
        if is_known_user and 'name' in self.user_models[user_id]:
            name = self.user_models[user_id]['name']
            greeting = f"{greeting} {name}!"
        else:
            greeting = f"{greeting}! I'm your assistant robot."
        
        return greeting
    
    def _select_greeting(self, is_known_user: bool, time_of_day: str) -> str:
        """
        Select appropriate greeting based on context
        """
        import random
        
        if not is_known_user:
            greetings = [
                "Hello",
                "Hi there",
                "Greetings",
                "Nice to meet you"
            ]
        else:
            if time_of_day == 'morning':
                greetings = ["Good morning", "Good to see you again this morning"]
            elif time_of_day == 'evening':
                greetings = ["Good evening", "Hello again"]
            else:
                greetings = ["Hello", "Hi", "Good to see you"]
        
        return random.choice(greetings)
    
    def manage_attention(self, current_user: str, detected_users: List[str]):
        """
        Manage attention between multiple users
        """
        if len(detected_users) == 1:
            # Single user interaction
            return current_user
        elif len(detected_users) == 0:
            # No users detected
            return None
        else:
            # Multiple users - implement social rules
            return self._handle_multiple_users(current_user, detected_users)
    
    def _handle_multiple_users(self, current_user: str, detected_users: List[str]) -> str:
        """
        Handle attention when multiple users are present
        """
        # Follow social rules for attention management
        # This might involve: politeness, turn-taking, etc.
        if current_user in detected_users:
            # Continue with current user
            return current_user
        else:
            # Switch to a new user
            # For now, pick the first one
            return detected_users[0]
    
    def generate_social_responses(self, context: Dict) -> Dict:
        """
        Generate socially appropriate responses
        """
        response_type = context.get('response_type', 'standard')
        
        responses = {
            'appreciation': [
                "Thank you for your patience.",
                "I appreciate your understanding.",
                "Thank you for working with me."
            ],
            'clarification': [
                "Could you please repeat that?",
                "I didn't quite catch that, could you say it again?",
                "I need a bit more information."
            ],
            'error_recovery': [
                "I apologize for the confusion.",
                "Let me try that again.",
                "I seem to have misunderstood. Can you help me?"
            ]
        }
        
        import random
        return responses.get(response_type, responses['appreciation'])
```

## 7.5 Practical Example: Complete Conversational System

Let's integrate all the components into a complete conversational robotics system:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Twist
from rclpy.qos import QoSProfile
import threading
import time

class ConversationalRobotNode(Node):
    def __init__(self):
        super().__init__('conversational_robot')
        
        # Initialize all system components
        self.multimodal_manager = MultimodalInteractionManager()
        self.social_engine = SocialRoboticsEngine()
        self.dialogue_state_tracker = DialogueStateTracker()
        
        # ROS 2 interfaces
        self.speech_subscriber = self.create_subscription(
            String, '/speech_input', self.speech_callback, 10
        )
        self.camera_subscriber = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.camera_callback, 10
        )
        self.speech_publisher = self.create_publisher(
            String, '/speech_output', 10
        )
        self.motion_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        # System state
        self.conversation_active = False
        self.current_user = None
        self.interaction_mode = 'attentive'  # attentive, reactive, idle
        
        # Start background processes
        self.multimodal_manager.start_listening()
        self.multimodal_manager.start_vision_processing()
        self.multimodal_manager.start_response_processing()
        
        # Timer for periodic tasks
        self.system_timer = self.create_timer(1.0, self.system_monitor)
        
        self.get_logger().info('Conversational Robot Node initialized')
    
    def speech_callback(self, msg):
        """
        Handle incoming speech messages
        """
        user_input = msg.data
        self.get_logger().info(f'Processing speech: {user_input}')
        
        # Process the speech input through our system
        response = self.process_conversation_turn(user_input)
        
        if response:
            # Publish the response
            response_msg = String()
            response_msg.data = response
            self.speech_publisher.publish(response_msg)
            
            # Also speak it out loud (in a real system, this would go to TTS)
            self.get_logger().info(f'Robot says: {response}')
    
    def camera_callback(self, msg):
        """
        Handle incoming camera data
        """
        # Process camera data for visual context
        # In a real system, this would convert ROS Image to OpenCV and process
        self.get_logger().debug('Received camera data')
        
        # Update visual context in multimodal manager
        # This would include object detection, face recognition, etc.
        pass
    
    def process_conversation_turn(self, user_input: str):
        """
        Process a complete conversation turn
        """
        # Add to dialogue history
        self.dialogue_state_tracker.conversation_history.append({
            'timestamp': time.time(),
            'speaker': 'user',
            'text': user_input,
            'intent': None,
            'entities': {}
        })
        
        # Process through dialogue system
        dialogue_result = self.multimodal_manager.dialogue_manager.process_input(user_input)
        
        intent = dialogue_result.get('intent', 'unknown')
        entities = dialogue_result.get('entities', {})
        
        # Update dialogue state
        self.dialogue_state_tracker.update_context(
            user_input=user_input,
            robot_response="",
            intent=intent,
            entities=entities
        )
        
        # Generate response based on intent and context
        response = self.generate_response(intent, entities, user_input)
        
        # Update conversation history with our response
        self.dialogue_state_tracker.conversation_history.append({
            'timestamp': time.time(),
            'speaker': 'robot',
            'text': response,
            'intent': intent,
            'entities': entities
        })
        
        # Execute any required actions
        self.execute_actions(intent, entities)
        
        return response
    
    def generate_response(self, intent: str, entities: Dict, user_input: str):
        """
        Generate response based on intent and context
        """
        if intent == 'greeting':
            # Use social engine to generate appropriate greeting
            context = {'time_of_day': 'day'}  # Would come from system clock
            return self.social_engine.handle_greeting(
                user_id=self.current_user or 'unknown', 
                context=context
            )
        elif intent == 'navigation':
            location = entities.get('location', 'destination')
            if location:
                return f"Okay, I'll navigate to the {location}. Please follow me."
            else:
                return "I can help you navigate, but I need to know where you'd like to go."
        elif intent == 'object_interaction':
            obj = entities.get('object', 'object')
            if obj:
                return f"I can help you with the {obj}. Can you show me where it is?"
            else:
                return "I can help you interact with objects. What would you like me to help you with?"
        else:
            return "I'm here to help. How can I assist you today?"
    
    def execute_actions(self, intent: str, entities: Dict):
        """
        Execute physical actions based on intent
        """
        if intent == 'navigation':
            location = entities.get('location')
            if location:
                self.navigate_to_location(location)
        elif intent == 'object_interaction':
            obj = entities.get('object')
            if obj:
                self.approach_object(obj)
    
    def navigate_to_location(self, location: str):
        """
        Execute navigation to a specific location
        """
        # In a real implementation, this would:
        # 1. Check if location is known
        # 2. Plan a path to the location
        # 3. Execute navigation commands
        # 4. Monitor progress
        self.get_logger().info(f'Navigating to {location}')
        
        # For simulation, send a simple movement command
        twist = Twist()
        twist.linear.x = 0.2  # Move forward at 0.2 m/s
        # In real system, this would be more sophisticated path following
        for _ in range(10):  # Move for 10 seconds as simulation
            self.motion_publisher.publish(twist)
            time.sleep(1.0)
        
        # Stop the robot
        stop_twist = Twist()
        self.motion_publisher.publish(stop_twist)
    
    def approach_object(self, obj_name: str):
        """
        Approach and interact with an object
        """
        self.get_logger().info(f'Approaching {obj_name}')
        # Implementation would involve:
        # - Locating the object in space
        # - Planning approach trajectory
        # - Executing approach maneuver
        pass
    
    def system_monitor(self):
        """
        Monitor system state and perform periodic tasks
        """
        # Check if conversation is still active
        # Reset if no recent activity
        if len(self.dialogue_state_tracker.conversation_history) > 0:
            last_entry = self.dialogue_state_tracker.conversation_history[-1]
            time_since_last = time.time() - last_entry['timestamp']
            
            # If no activity in 30 seconds, consider conversation ended
            if time_since_last > 30 and self.conversation_active:
                self.conversation_active = False
                self.current_user = None
                self.interaction_mode = 'idle'
                self.get_logger().info('Conversation ended due to inactivity')
    
    def shutdown(self):
        """
        Clean shutdown of the system
        """
        self.get_logger().info('Shutting down conversational robot')
        # Stop all background processes
        # Save conversation history
        # Clean up resources

def main(args=None):
    rclpy.init(args=args)
    
    try:
        robot = ConversationalRobotNode()
        
        # Run the node
        rclpy.spin(robot)
        
    except KeyboardInterrupt:
        pass
    finally:
        robot.shutdown()
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 7.6 Summary

This chapter has explored the complex field of conversational robotics, which integrates natural language processing, speech recognition, dialogue management, and multimodal interaction to create robots that can engage in human-like conversations. Key takeaways include:

- Conversational AI in robotics combines ASR, NLU, dialogue management, NLG, and TTS for natural interaction
- Effective dialogue management requires state tracking, intent recognition, and context awareness
- Multimodal interaction integrates speech, vision, and action for more natural communication
- Social robotics principles help create engaging and appropriate robot behavior
- Real conversational systems require integration of multiple complex components

Conversational robotics represents a critical component of Physical AI systems, enabling more natural and intuitive human-robot interaction.

## 7.7 Exercises

### Exercise 7.1: Speech Recognition System
Implement a robust speech recognition system that handles noise cancellation and keyword spotting for robot activation.

### Exercise 7.2: Dialogue State Tracker
Create a dialogue state tracker that maintains conversation context across multiple turns and sessions.

### Exercise 7.3: Intent Recognition
Build an intent recognition system that accurately identifies user intents and extracts relevant entities.

### Exercise 7.4: Multimodal Integration
Develop a system that combines speech and visual input to improve understanding and response generation.

### Exercise 7.5: Complete Conversational Agent
Build a complete conversational robot system that handles listening, processing, and responding appropriately.