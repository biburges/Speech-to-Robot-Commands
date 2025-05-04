# Instructions(ONLY ON LINUX) 

## Prerequisites 

Make sure the following are installed and working
- ROS 1
- TurtleBot3 Packages
- Gazebo
- Ollama running with llama3.2:latest
- Vosk

Additionally make sure you have the latest python script downloaded and placed into the ``IE Final Project/Speach_to_text_ollama`` folder

Now you will need four terminals open

## Terminal 1 

Start ROS Core

```
roscore
```

## Terminal 2

Launch TurtleBot3 Simualtion

```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
## Terminal 3

Start Ollama Server

```
ollama serve
```

Then, make sure your model is available

```
ollama run llama3.2
```

If it's not pulled yet

```
ollama pull llama3.2
```

## Terminal 4

Change directories to the folder that has the python script and run the script

```
python3 final_voice_ollama.py
```

## While it's running

When the simulation is generated on your screen and the code is running, you can speak commands to it. You can say things such as "move forward" or "stop"
