import os
import sys
import time
import signal
import json
import pyaudio
import requests
import pyttsx3
import rospy
from vosk import Model, KaldiRecognizer
from geometry_msgs.msg import Twist

# === Text-to-Speech Engine ===
engine = pyttsx3.init()

# === Graceful Exit ===
def exit_gracefully(signum, frame):
    print("\nExiting. Goodbye!")
    engine.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, exit_gracefully)

# === Load Vosk Speech Recognition Model ===
def load_vosk_model():
    model_path = "model/vosk-model-small-en-us-0.15"
    if not os.path.exists(model_path):
        raise FileNotFoundError("Vosk model not found! Please ensure it's placed in the correct directory.")
    return Model(model_path)

# === Listen and Transcribe User Voice Input ===
def listen_and_transcribe(model, timeout=3):
    rec = KaldiRecognizer(model, 16000)
    audio = pyaudio.PyAudio()
    stream = audio.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=16000,
        input=True,
        frames_per_buffer=2048
    )
    stream.start_stream()

    print("Listening... Speak now.")
    start_time = time.time()
    text = ""

    while True:
        data = stream.read(2048, exception_on_overflow=False)
        if rec.AcceptWaveform(data):
            result = rec.Result()
            text = json.loads(result).get('text', '')
            if text:
                break
        if time.time() - start_time > timeout:
            print("Listening timed out.")
            break

    stream.stop_stream()
    stream.close()
    audio.terminate()
    return text

# === Query Ollama AI ===
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
        response = requests.post(url, json=payload, stream=True, timeout=10)
        response.raise_for_status()

        full_response = ""
        for line in response.iter_lines(decode_unicode=True):
            if line:
                data = json.loads(line)
                message = data.get("message", {}).get("content", "")
                full_response += message
                if "`/cmd_vel" in message or "/cmd_vel" in message:
                    break
        return full_response.strip()

    except Exception as e:
        return f"Error: {e}"

# === Speak Response ===
def speak_text(text):
    engine.say(text)
    engine.runAndWait()

# === Convert Command String to ROS Twist ===
def parse_cmd_vel_to_twist(cmd_str):
    twist = Twist()
    try:
        # Clean and extract values
        line = [line for line in cmd_str.split('\n') if '/cmd_vel' in line]
        if line:
            parts = line[0].split(":")[1].strip().split()
            twist.linear.x = float(parts[0])
            twist.linear.y = float(parts[1])
            twist.angular.z = float(parts[2])
    except Exception as e:
        print(f"Failed to parse /cmd_vel command: {e}")
    return twist

# === Initialize ROS ===
def init_ros():
    rospy.init_node('voice_ollama_controller', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    return pub

# === Main Function ===
def main():
    try:
        model = load_vosk_model()
        pub = init_ros()

        while not rospy.is_shutdown():
            user_input = listen_and_transcribe(model)
            if not user_input:
                continue

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

