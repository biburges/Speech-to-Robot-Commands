
# Speech to Robot Commands

Team Members:
- Bianca Burgess, biburges@buffalo.edu
- Piyush Salian, psalian@buffalo.edu

--- 

## Project Objective
The goal of this project is to take speech and convert it into text which will convert it into robot commands for a robot car.


## Contributions
This project is unique as it will allow people with upper limb disabilities to be able to use and provide commands to robot cars.


## Project Plan

### Speech-to-Text

Using Python Speech Recognition software, PyAudio, and Python pyttsx3, it will take input from a microphone and convert it into text

### LLM

Using a LLM such as OpenAi or Keyword Matching, to take the text and convert it into commands for the robot to understand

### Robot Commands

We will use ROS or Gazebo to take the robot commands and send them to the robot car.

## Milestones/Schedule Checklist
{What are the tasks that you need to complete?  Who is going to do them?  When will they be completed?}
- [x] Complete this proposal document.  *Due Feb. 28*
- [ ] Successfully setup speechrecognition python speech recog. module *Due April 7*
- [ ] Successfully extract text from speech with as much noise dampening as possible *Due April 10*
- [ ] Successfully setup an LLM *Due April 12*
- [ ] Successfully interface LLM with output from Python Speech recognition via live feed *Due April 13*
- [ ] Setup Gazebo successfully *Due April 17*
- [ ] Setup simulated robot car *Due April 19*
- [ ] Interface LLM outputs with Gazebo topics for car movement *Due April 30*
- [ ] Create final presentation.  *Due May 6*
- [ ] Provide system documentation (README.md).  *Due May 13*


## Measures of Success
We believe partial credit should come from the achievement of major milestones in the form of 
1) Succesfully setting up SpeechRecognition and pyttsx3 library of Python to extract live feed audio input
2) Setting up LLM to produce robot commands specific to robot args and DOF given a complex life feed audio input
3) Setting up the gazebo simulation to interface with LLM 
4) Complete simulation responds effectively to voice input
