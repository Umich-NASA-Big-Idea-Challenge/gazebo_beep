# gazebo_beep

## Setup
This repo holds the files needed to simulate the cargo beep in a gazebo env.

to get the sim set up see: https://github.com/Umich-NASA-Big-Idea-Challenge/beep_sim 

#### Cloning
you should clone this repo inside the sim env

if you have the remote explorer extension in vs code you can enter the container and use the git credentials of your computer or you can go through the process of setting up a ssh token in the docker container

#### Source Ros
echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc
source ~/.bashrc 

### Recreate the test_ws
I am not typing that out you know how it is what is on the robot

### Starting the sim
the command to start a sim is 

ign gazebo name_of_file.sdf

open another terminal in the docker env and run launch.sh in the launch folder this establish a bridge in betwen ros and gazebo so they can talk to each other

Note beep_start is not need as the sim takes the place of beep_start so beep_build and then run whatever you want


