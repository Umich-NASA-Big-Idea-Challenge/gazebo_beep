# gazebo_beep

This repo holds the files needed to simulate the cargo beep in a gazebo env.

to get the sim set up see: https://github.com/Umich-NASA-Big-Idea-Challenge/beep_sim 

you should clone this repo inside the sim env

the command to start a sim is 

ign gazebo name_of_file.sdf

open another terminal in the docker env and run launch.sh in the launch folder this establish a bridge in betwen ros and gazebo so they can talk to eachother

