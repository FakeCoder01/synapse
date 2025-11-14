"""
Audio Interface Node
Handles text-to-speech (TTS) output and speech-to-text (STT) input
"""

import rclpy
from rclpy.node import Node
import threading
import queue
import json

from std_msgs.msg import String
from sensor_msgs.msg import Audio

try:
    import pyttsx3
    TTS_AVAILABLE = True
except ImportError:
    TTS_AVAILABLE = False

try:
    import speech_recognition as sr
    STT_AVAILABLE = True
except ImportError:
    STT_AVAILABLE = False

try:
    import pyaudio
    PYAUDIO_AVAILABLE = True
except ImportError:
    PYAUDIO_AVAILABLE = False


class AudioInterfaceNode(Node):
    """
    Audio interface for TTS and STT
    
    Subscriptions:
    - /robot/tts (String): Text to speak
    - /robot/audio (String): Audio commands
    
    Publications:
    - /robot/audio_transcription (String): Speech-to-text results
    - /robot/audio_event (String): Audio events (recording, etc.)
    """

    def __init__(self):
        super().__init__('audio_interface')
        
        # Parameters
        self.declare_parameter('enable_tts', True)
        self.declare_parameter('enable_stt', True)
        self.declare_parameter('tts_rate', 150)
        self.declare_parameter('tts_volume', 1.0)
        self.declare_parameter('stt_language', 'en-US')
        self.declare_parameter('audio_device_index', None)
        
        self.enable_tts = self.get_parameter('enable_tts').value
        self.enable_stt = self.get_parameter('enable_stt').value
        self.tts_rate = self.get_parameter('tts_rate').value
        self.tts_volume = self.get_parameter('tts_volume').value
        self.stt_language = self.get_parameter('stt_language').value
        self.audio_device_index = self.get_parameter('audio_device_index').value
        
        # Validate availability
        if self.enable_tts and not TTS_AVAILABLE:
            self.get_logger().warn('pyttsx3 not available. Install: pip install pyttsx3')
            self.enable_tts = False
        
        if self.enable_stt and not STT_AVAILABLE:
            self.get_logger().warn(
                'speech_recognition not available. Install: pip install SpeechRecognition'
            )
            self.enable_stt = False
        
        # Initialize TTS
        if self.enable_tts:
            self._init_tts()
        
        # Initialize STT
        if self.enable_stt:
            self._init_stt()
        
        # Subscriptions
        self.tts_subscription = self.create_subscription(
            String,
            '/robot/tts',
            self._tts_callback,
            10
        )
        
        self.audio_cmd_subscription = self.create_subscription(
            String,
            '/robot/audio_cmd',
            self._audio_cmd_callback,
            10
        )
        
        # Publishers
        self.transcription_publisher = self.create_publisher(
            String, '/robot/audio_transcription', 10
        )
        self.audio_event_publisher = self.create_publisher(
            String, '/robot/audio_event', 10
        )
        
        # Thread-safe queue for TTS
        self.tts_queue = queue.Queue()
        
        # STT thread
        if self.enable_stt:
            self.stt_running = True
            self.stt_thread = threading.Thread(target=self._stt_loop, daemon=True)
            self.stt_thread.start()
        
        # TTS processing thread
        if self.enable_tts:
            self.tts_thread = threading.Thread(target=self._tts_loop, daemon=True)
            self.tts_thread.start()
        
        self.get_logger().info(
            f'Audio Interface initialized. TTS: {self.enable_tts}, STT: {self.enable_stt}'
        )

    def _init_tts(self):
        """Initialize text-to-speech engine"""
        try:
            self.tts_engine = pyttsx3.init()
            self.tts_engine.setProperty('rate', self.tts_rate)
            self.tts_engine.setProperty('volume', self.tts_volume)
            self.get_logger().info('TTS engine initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize TTS: {e}')
            self.enable_tts = False

    def _init_stt(self):
        """Initialize speech-to-text"""
        try:
            self.recognizer = sr.Recognizer()
            self.microphone = sr.Microphone(device_index=self.audio_device_index)
            
            # Adjust for ambient noise
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
            
            self.get_logger().info('STT engine initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize STT: {e}')
            self.enable_stt = False

    def _tts_callback(self, msg: String):
        """Handle TTS requests"""
        if not self.enable_tts:
            return
        
        text = msg.data
        self.get_logger().info(f'TTS request: {text}')
        
        # Queue the text for processing
        self.tts_queue.put(text)

    def _tts_loop(self):
        """Process TTS queue in background"""
        while True:
            try:
                text = self.tts_queue.get(timeout=0.5)
                self.tts_engine.say(text)
                self.tts_engine.runAndWait()
                self.get_logger().debug(f'TTS spoke: {text}')
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().error(f'TTS error: {e}')

    def _stt_loop(self):
        """Continuous speech recognition loop"""
        while self.stt_running and self.enable_stt:
            try:
                self._listen_and_transcribe()
            except Exception as e:
                self.get_logger().error(f'STT error: {e}')

    def _listen_and_transcribe(self):
        """Listen to microphone and transcribe speech"""
        try:
            with self.microphone as source:
                self.get_logger().debug('Listening for speech...')
                
                # Record audio
                audio = self.recognizer.listen(
                    source,
                    timeout=5.0,
                    phrase_time_limit=10.0
                )
            
            # Transcribe using Google Speech Recognition
            try:
                text = self.recognizer.recognize_google(
                    audio,
                    language=self.stt_language
                )
                
                # Publish transcription
                msg = String()
                msg.data = text
                self.transcription_publisher.publish(msg)
                
                self.get_logger().info(f'Transcribed: {text}')
                
                # Publish event
                event_msg = String()
                event_msg.data = json.dumps({
                    'type': 'transcription',
                    'confidence': 'high',
                    'text': text
                })
                self.audio_event_publisher.publish(event_msg)
                
            except sr.UnknownValueError:
                self.get_logger().debug('Could not understand audio')
            except sr.RequestError as e:
                self.get_logger().error(f'STT service error: {e}')
                
        except Exception as e:
            self.get_logger().debug(f'Listening error: {e}')

    def _audio_cmd_callback(self, msg: String):
        """Handle audio commands"""
        try:
            cmd = json.loads(msg.data)
            
            if cmd.get('command') == 'buzz':
                self._play_buzz(cmd.get('frequency', 1000), cmd.get('duration', 100))
            elif cmd.get('command') == 'beep':
                self._play_beep()
            elif cmd.get('command') == 'start_listening':
                self.get_logger().info('Starting continuous listening')
            elif cmd.get('command') == 'stop_listening':
                self.get_logger().info('Stopping listening')
        except Exception as e:
            self.get_logger().error(f'Audio command error: {e}')

    def _play_buzz(self, frequency: int, duration: int):
        """Play a buzzing sound (requires speaker wired to GPIO)"""
        # This would be implemented on Raspberry Pi with GPIO PWM
        self.get_logger().info(f'Buzz: {frequency} Hz for {duration} ms')
        
        event_msg = String()
        event_msg.data = json.dumps({
            'type': 'buzz',
            'frequency': frequency,
            'duration': duration
        })
        self.audio_event_publisher.publish(event_msg)

    def _play_beep(self):
        """Play a beep sound"""
        self.get_logger().info('Beep!')
        
        if self.enable_tts:
            self.tts_queue.put('beep')

    def destroy_node(self):
        """Cleanup"""
        if self.enable_stt:
            self.stt_running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()