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
from sensor_msgs.msg import LaserScan
import time
import math

# Initialize TTS engine globally to prevent garbage collection issues
engine = pyttsx3.init()

# Global publisher and obstacle flag
pub = None
obstacle_detected = False

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

# Listen to microphone input and transcribe using Vosk with timeout
def listen_and_transcribe(model, timeout=10):
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
    start_time = time.time()

    while True:
        data = stream.read(4096)
        if rec.AcceptWaveform(data):
            result = rec.Result()
            text = eval(result).get('text', '')
            if text:
                return text
        if time.time() - start_time > timeout:
            print("Listening timed out.")
            return ""

# Send the transcribed text to Ollama and get a response with timeout
def query_ollama(input_text, timeout=10):
    url = "http://localhost:11434/api/chat"
    payload = {
        "model": "llama3.2:latest",
        "messages": [
            {
                "role": "system",
                "content": "You are a ROS expert. Convert user commands into ONLY ROS TurtleBot3 motion commands in Python. Respond with:\n- `/move: speed distance angular` for linear movement,\n- `/rotate: angular_speed angle_in_radians` for turning in place.\nIf the command is 'stop', respond with `/cmd_vel: 0 0 0`. Use only **1.0 m/s** for linear speed and **1.5708 rad/s** for angular speed. Do not change these speeds. Use raw numeric values only. Do not include units like rad in the output."
            },
            {
                "role": "user",
                "content": input_text
            }
        ]
    }

    try:
        response = requests.post(url, json=payload, stream=True, timeout=30)
        response.raise_for_status()

        full_response = ""
        for line in response.iter_lines(decode_unicode=True):
            if line:
                data = json.loads(line)
                message = data.get("message", {}).get("content", "")
                full_response += message
                if "/cmd_vel" in message or "/move" in message or "/rotate" in message:
                    break
        return full_response.strip()

    except requests.exceptions.Timeout:
        return "Error: Ollama API request timed out."
    except Exception as e:
        return f"Error: {e}"

# Convert Ollama's response to speech
def speak_text(text):
    engine.say(text)
    engine.runAndWait()

# Parse /cmd_vel into Twist
def parse_cmd_vel_to_twist(cmd_str):
    twist = Twist()
    try:
        cmd_str = cmd_str.replace('`', '').strip()
        lines = cmd_str.splitlines()
        cmd_line = next((line for line in lines if line.strip().startswith('/cmd_vel:')), None)
        if not cmd_line:
            raise ValueError("No /cmd_vel: line found.")
        parts = cmd_line.split(":")[1].strip().split()
        twist.linear.x = float(parts[0])
        twist.linear.y = float(parts[1])
        twist.angular.z = float(parts[2])
    except Exception as e:
        print(f"Failed to parse command: {e}")
    return twist

# Helper for evaluating math expressions
def convert(val):
    val = val.replace('π', 'pi')  # support both π and pi
    try:
        return eval(val, {"__builtins__": None}, {"math": math, "pi": math.pi})
    except Exception as e:
        print(f"Error evaluating expression {val}: {e}")
        return 0

# Parse /move command
def parse_move_command(cmd_str):
    try:
        cmd_str = cmd_str.replace('`', '').strip()
        lines = cmd_str.splitlines()
        move_line = next((line for line in lines if line.strip().startswith('/move:')), None)
        if not move_line:
            raise ValueError("No /move: line found.")
        parts = move_line.split(":")[1].strip().split()

        speed = float(parts[0])
        distance = parts[1]
        if "inch" in distance.lower():
            distance = convert(distance.replace('inch', '')) * 0.0254
        else:
            distance = convert(distance)
        angular = convert(parts[2]) if len(parts) > 2 else 0.0
        return speed, distance, angular
    except Exception as e:
        print(f"Failed to parse /move command: {e}")
        return None, None, None

# Parse /rotate command
def parse_rotate_command(cmd_str):
    try:
        cmd_str = cmd_str.replace('`', '').strip()
        lines = cmd_str.splitlines()
        rotate_line = next((line for line in lines if line.strip().startswith('/rotate:')), None)
        if not rotate_line:
            raise ValueError("No /rotate: line found.")
        parts = [p.replace('rad', '').strip() for p in rotate_line.split(":")[1].strip().split()]
        angular_speed = float(parts[0])
        angle_rad = eval(parts[1].replace('π', 'pi'), {"__builtins__": None}, {"pi": math.pi})
        return angular_speed, angle_rad
    except Exception as e:
        print(f"Failed to parse /rotate command: {e}")
        return None, None

# Move robot a given distance at given speed
def move_distance(pub, speed, distance, angular=0.0):
    global obstacle_detected

    # Ensure correct direction handling
    direction = math.copysign(1, distance)
    linear_speed = direction * abs(speed)

    # For backward movement, apply negative speed
    if distance < 0:
        print("Moving backward.")
        linear_speed = -abs(linear_speed)  # Move backward by applying negative speed
    else:
        print("Moving forward.")
        linear_speed = abs(linear_speed)  # Move forward by applying positive speed

    # Check for obstacles if moving forward
    if obstacle_detected and linear_speed > 0:
        print("Too close. Backing off.")
        speak_text("Obstacle detected. Reversing a little.")
        back_twist = Twist()
        back_twist.linear.x = -0.1
        for _ in range(10):
            pub.publish(back_twist)
            rospy.sleep(0.1)
        pub.publish(Twist())
        return

    # If no obstacle, move with desired speed and angular velocity
    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = angular

    # Duration of the movement
    duration = abs(distance / speed) if speed != 0 else 0
    start_time = time.time()
    rate = rospy.Rate(10)

    print(f"Moving at speed: {linear_speed} m/s for {distance} meters.")

    while not rospy.is_shutdown() and (time.time() - start_time) < duration:
        if obstacle_detected and linear_speed > 0:
            print("Obstacle detected during movement. Stopping.")
            break
        pub.publish(twist)
        rate.sleep()

    pub.publish(Twist())
    print(f"Movement completed for {distance} meters.")



# Rotate in place
def rotate_in_place(pub, angular_speed, angle_rad):
    twist = Twist()
    twist.angular.z = angular_speed if angle_rad >= 0 else -abs(angular_speed)

    duration = abs(angle_rad / angular_speed) if angular_speed != 0 else 0
    start_time = time.time()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown() and (time.time() - start_time) < duration:
        pub.publish(twist)
        rate.sleep()

    pub.publish(Twist())

# Obstacle detection callback
def laser_callback(data):
    global obstacle_detected
    try:
        valid_ranges = [r for r in data.ranges if 0.05 < r < 3.0]
        if valid_ranges:
            front = min(min(valid_ranges[0:10]), min(valid_ranges[-10:]))
            obstacle_detected = front < 0.5
        else:
            obstacle_detected = True
    except Exception as e:
        print(f"Laser error: {e}")
        obstacle_detected = True

# Initialize ROS and publisher
def init_ros():
    global pub
    rospy.init_node('voice_ollama_controller', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    return pub

# Main function
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
                if not obstacle_detected or twist_msg.linear.x <= 0:
                    pub.publish(twist_msg)
                    print("Published to ROS /cmd_vel!")
                else:
                    print("Obstacle ahead. Command blocked.")
                    speak_text("There's an obstacle in front.")
            elif "/move" in response:
                speed, distance, angular = parse_move_command(response)
                if speed is not None:
                    move_distance(pub, speed, distance, angular)
                    print(f"Moved {distance} meters at {speed} m/s.")
            elif "/rotate" in response:
                angular_speed, angle_rad = parse_rotate_command(response)
                if angular_speed is not None:
                    rotate_in_place(pub, angular_speed, angle_rad)
                    print(f"Rotated {angle_rad} radians at {angular_speed} rad/s.")

    except Exception as e:
        print(f"An error occurred: {e}")
        engine.stop()

if __name__ == "__main__":
    main()
