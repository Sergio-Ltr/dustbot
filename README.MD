# Dustbot

Simulation of a robot cleaner moving in a 2D grid. 

## Requirements

To run the simulation make sure ypu have installed a ros distribution and a catkin workspace is setted. 
During the development the "noetic" version of ROS, installed on Ubuntu 20.4 has been used. 

## Importing the code

To import the code, cd into your catkin workspace. In my case it would be:

```
cd ~/catkin_ws
```

Then move into the `src` directory and clone the repository.

```
cd src
git clone https://github.com/Sergio-Ltr/dustbot.git
```

## Executing 

Make sure roscore is up on any terminal, then, you counld start dustbot using the launch file

```
roslaunch dustbot dustbot.launch
```

Alternatively, you can run the two nodes on two separate terminals: 

```
rosrun dustbot world.py
rosrun dustbot robot.py
```

## What to expect as output

The simulation only produces a series of logs. 

The majority of logs from the `/world` node are in `UPPERCASE`, while logs from `/robot` node are all in `lowecase` and generaly starts with the `/robot_node` prefix. 

Execution is a little bit verbose, but reading all the messages the adventures of our robot can be followed pretty easily. 

## Two lines about the architecture

The system consists in two nodes: 

- /robot
- /world

The world node controls the environment (and actually moves the robot) while the robot node contains the logic behind the "decisions" taken. 

Robot is supposed to move in a 2D N x N grid, at the speed of one cell at each second, in one of four possible directions (N,S,W,E), which the robot node can require to change by a service. 

The robot position is updated by the `/world node` at each second, being published on the `global_position` topic. 

World node also publishes the coordinates of the next piece of garbage to be picked on the `current_destination` topic. 

Both topics are subscribed by the robot node. When the cell containg garbage is reached, another service `load_garbage`, is called, in order to effectively clean the cell.
Once the request is served by the world node, a new garbage cell is randomly chosen and published, so that the loop can contiune, up to P times. 

Values for N and P are setted in the `\config\grid.yaml` file. They can safely be modified for further simulations. 
