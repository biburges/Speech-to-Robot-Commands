# Speech to Robot Commands
Bianca Burgess (biburges@buffalo.edu) & Piyush Salian (psalian@buffalo.edu)

## Motivation / Overview of your project.
#### Why did you choose this project? At a high level, what is your project about? Who should care about what you've done?
Modern day human robot interaction is severly dependent on the user's ability to utilize specific mediums for input. For example, manipulating robot motion via gazebo is predominately achieved via the passing of commands to the cmd/vel topic by keyboard input. This makes the method inaccessible to those who are differently abled. Birthed from this observation, our project hopes to address that inaccesibility by voice controlled robot manipulation. 

This is achieved by the recognition and analysis of voice commands, the parsing of the input through a LLM and ultimately the production of topic messages sent directly to a ROS node for robot manipulation. Steps taken to increase accessiblity include the choosing of specific voice recognition model that accomodate different accents and LLMs that account for the user's spread in technical knowledge regarding ROS topic messages. 

## Demonstration

See the video for demonstration

[![Demo](thumb.jpg)](https://vimeo.com/1081331997/fba82ef947?ts=0&share=copy)

### Examples

**Example 1: Turning in Place 90 Degrees**

Voice Command: "Turn 90 degrees"

Ollama Output for ROS: `/rotate: 1 pi/2`

Results in Simulation: Rotate 1.57 radians counterclockwise at 1.57 rad/s

**Example 2: Move Forward 1 Meter**

Voice Command: "Move forward 1 meter"

Ollama Output for ROS: `move/: 1.0 1.0 1.0`

Results in Simulation: Moved 1.0 meters at 0.5 m/s

**Example 3: Move Backward 1 Meter**

Voice Command: "Move backward 1 meter"

Ollama Output for ROS: `move/: 1.0 -1.0 1.0`

Results in Simulation: Moved -1.0 meter at 0.5 m/s

## Visual Display of Data Flow through Functions
![Image](Function_Flowchart.png)


## Installation Instructions

- Download and Install VOSK on your machine
```    
pip3 install vosk
 ```
- Installing the portaudio development library
```
sudo apt update
sudo apt install portaudio19-dev python3-dev
```
- Install pyaudio on your machine
```
pip install pyaudio
```
- Install requests on your machine
```      
pip install requests
```
- Install pyttxs3 on your machine
```
pip install pyttsx3
```
- Install Ollama Version 0.6.3 (https://ollama.com/download)
- Verify Installation via command line interfeace (windows command prompt)
```
ollama --version
```
- View any current ollama processes via:
```
tasklist | findstr /i "ollama"
```
- SKIP THIS STEP IF NOT APPLICABLE: Remove any processes that are active, where the PID is the code written before "CONSOLE" on the CLI:
```
taskkill /PID <PID> /F
```

## How to Run the Code (Linux)


#### Prerequisites 

Make sure the following are installed and working
- ROS 1
- TurtleBot3 Packages
- Gazebo
- Ollama running with llama3.2:latest
- Vosk

Additionally make sure you have the latest python script downloaded and placed into the ``IE Final Project/Speach_to_text_ollama`` folder

Now you will need about three terminals open

### Terminal 1

Launch TurtleBot3 Simualtion

```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
### Terminal 2

Start Ollama Server

```
ollama serve
```

Then, make sure your model is available

> Troubleshooting Tip: If running ollama serve produces an error like "Error: listen tcp ABC.0.0.1:XYZEF: bind: address already in use", run the following code
> ```
> sudo systemctl stop ollama
> ```

**If you are not able to run this in Terminal 2, open a new terminal**

```
ollama run llama3.2
```

If it's not pulled yet

```
ollama pull llama3.2
```

### Terminal 3

Change directories to the folder that has the python script and run the script

```
python3 final_voice_ollama.py
```

### While it's running

When the simulation is generated on your screen and the code is running, you can speak commands to it. You can say things such as "move forward" or "stop"

**See the ![video](How_To_Run.mp4) to see how to run it**


## References
Include links to websites you found helpful.
Also mention websites you tried but were not as helpful

- Build a Voice Assistant with Ollama: Speech-to-Text and TTS_ Episode 2
( https://www.youtube.com/watch?v=cMDHTXobwxk&list=PLuxh5fnYpxkGz7PnCX_HDE4XSMri9vp6K&index=5)
- Learn Ollama in 15 Minutes - Run LLM Models Locally for FREE
(https://www.youtube.com/watch?v=UtSSMs6ObqY)
- How to Install & Use Whisper AI Voice to Text
(https://www.youtube.com/watch?v=ABFqbY_rmEk&t=328s)
- Best FREE Speech to Text AI - Whisper AI
(https://www.youtube.com/watch?v=8SQV-B83tPU)


## Future Work
1) If you had more time, what would you do with this project?
2) Are there some bugs you need to fix? Please document where these are, what you've tried to do to fix them, and suggestions you have for how these could be fixed by someone else.
3) Are there new features you'd add? Please provide as many details as possible.

More time for this project would enable us to explore multiple features within the turtlebot simulator, which pushes forth our overall theme of technological accessiblity. Our current iteration includes the manipulation of robot velocity and orientation via the cmd/vel topic. However, there exist multiple other topics like /scan and /odom. Initializng a 2 way communication between ROS topics and the LLM (instead of the current one way) would allow the user to view topic inputs as well as dictate topic outputs. This manifests in dynamic interaction with the environment and the passing of prompts like "go towards the closest object). 

The addition of the autonomous waypoint navigation through the incorporation of slam gmapping. This would enable the user to simply pass waypoints that the robot moves towards, without the interacting with the cmd/vel topic. Users can then choose the features of the world they spawn in and pass commands to proceed towards or away from the landmarks.

Also, we would like to include a way for the user change the speed through speech. Currently, we have it programmed so it will move at a velocity of 0.5 m/s and turn at a rate of 1.57 rad/s. We would like to continue to find a way to change the speed while the robot is moving.
