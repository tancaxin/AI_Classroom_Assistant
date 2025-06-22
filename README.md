# AI Classroom Assistant
An interactive, voice-controlled robot tutor built with ROS that helps young children learn basic concepts like **colours** and **shapes** through visual object recognition and conversation. Children can show objects to the robot’s camera and ask questions like _“What colour is this?”_ or _“What shape is this?”_. The robot will respond with a friendly explanation using TTS (Text-to-Speech).

## Development Environment
- Ubuntu 20.04
- ROS Noetic
- Python >=3.10.0

## Project Setup
### Install Anaconda
1. Download the corresponding installation package [Anaconda](https://www.anaconda.com/download#linux)
2. Install Anaconda
   
    ``` $ bash ~/Downloads/Anaconda3-2024.10-1-Linux-x86_64.sh ```

3. Find the .bashrc file, go to the home folder and press ctrl+h
4. Edit the ~/.bashrc file and add the following PATH export to the end of the file

    ``` export PATH=/home/mustar/anaconda3/bin:$PATH ```

6. Source the ~/.bashrc file after saving and exit terminal
   
    ``` $ source ~/.bashrc ```

### Create Virtual Environment
1. Check if conda is installed and in your PATH
   1. Open a terminal
   2. Enter ``` conda -V ``` into the terminal command line and press enter.
   3. If conda is installed you should see something like the following.
      ```
      $ conda -V
      conda 3.7.0
      ```

2. Create a new virtual environment

   ``` $ conda create -n yourenvname python=3.10 anaconda ```

3. Activate the virtual environment

   ``` $ conda activate yourenvname ```

### Clone This Repository
```
$ cd catkin_ws/src
$ git clone https://github.com/tancaxin/AI_Classroom_Assistant.git
$ cd catkin_ws
$ catkin_make
```
### Install Required Libraries
``` 
$ conda activate yourenvname
$ cd catkin_ws/src/AI_Classroom_Assistant
$ pip install -r requirements.txt
```

### Run This Project
1. Setup OpenAI API key<br>
   1. Open ``` recognition_controller.py ```.
   2. Find the line with "API_KEY" replace it with the actual API key provided.
   3. Save the file.
2. Open a terminal, and run the following code:
```
$ roslaunch ai_assistant ai_assistant.launch
```
#### OR
Open 5 separate terminals, and run the following commands (one per terminal).
```
$ conda activate yourenvname
$ roscore
```
```
$ conda activate yourenvname
$ roslaunch usb_cam usb_cam-test.launch
```
```
$ conda activate yourenvname
$ rosrun ai_assistant google_sr.py
```
```
$ conda activate yourenvname
$ rosrun ai_assistant recognition_controller.py
```
```
$ conda activate yourenvname
$ rosrun ai_assistant google_tts.py
```
         
