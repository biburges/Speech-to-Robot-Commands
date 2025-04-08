
import os
import pyaudio
from vosk import Model, KaldiRecognizer
import requests
import pyttsx3
import json
import signal
import sys
import rospy
from geometry_msgs.msg import Twist

# Initialize TTS engine globally to prevent garbage collection issues
engine = pyttsx3.init()

# Handle graceful exit on Ctrl+C
def exit_gracefully(signum, frame):
    print("\nExiting. Goodbye!")
    engine.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, exit_gracefully)

# Load Vosk model for offline speech recognition
def load_vosk_model():
    model_path = "model/vosk-model-small-en-us-0.15"
    if not os.path.exists(model_path):
        raise FileNotFoundError("Vosk model not found! Download and place it in the 'models' directory.")
    return Model(model_path)

# Listen to microphone input and transcribe using Vosk
def listen_and_transcribe(model):
    rec = KaldiRecognizer(model, 16000)
    audio = pyaudio.PyAudio()
    stream = audio.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=16000,
        input=True,
        frames_per_buffer=8192
    )
    stream.start_stream()

    print("Listening... Speak now.")
    while True:
        data = stream.read(4096)
        if rec.AcceptWaveform(data):
            result = rec.Result()
            text = eval(result).get('text', '')
            if text:
                return text

# Send the transcribed text to Ollama and get a response
def query_ollama(input_text):
    url = "http://localhost:11434/api/chat"
    payload = {
        "model": "llama3.2:latest",
        "messages": [
            {
                "role": "system",
                "content": "You are a ROS expert. Convert user commands into ROS TurtleBot3 motion commands in Python. OUTPUT ONLY COMMANDS. If the command is 'stop', set all velocities to 0. Use format: `/cmd_vel: x y z`."
            },
            {
                "role": "user",
                "content": input_text
            }
        ]
    }

    try:
        response = requests.post(url, json=payload, stream=True)
        response.raise_for_status()

        full_response = ""
        for line in response.iter_lines(decode_unicode=True):
            if line:
                data = json.loads(line)
                message = data.get("message", {}).get("content", "")
                full_response += message
                if "`/cmd_vel" in message:
                    break
        return full_response.strip()

    except Exception as e:
        return f"Error: {e}"

# Convert Ollama's response to speech
def speak_text(text):
    engine.say(text)
    engine.runAndWait()

# Parse command string into ROS Twist message
def parse_cmd_vel_to_twist(cmd_str):
    twist = Twist()
    try:
        parts = cmd_str.split(":")[1].strip().split()
        twist.linear.x = float(parts[0])
        twist.linear.y = float(parts[1])
        twist.angular.z = float(parts[2])
    except Exception as e:
        print(f"Failed to parse command: {e}")
    return twist

# Initialize ROS and publisher
def init_ros():
    rospy.init_node('voice_ollama_controller', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    return pub

# Main function
def main():
    try:
        model = load_vosk_model()
        pub = init_ros()

        while not rospy.is_shutdown():
            user_input = listen_and_transcribe(model)
            print(f"You said: {user_input}")

            response = query_ollama(user_input)
            print(f"Ollama: {response}")
            speak_text(response)

            if "/cmd_vel" in response:
                twist_msg = parse_cmd_vel_to_twist(response)
                pub.publish(twist_msg)
                print("Published to ROS /cmd_vel!")

    except Exception as e:
        print(f"An error occurred: {e}")
        engine.stop()

if __name__ == "__main__":
    main()
