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

An exaple of output executing dustbot by lauch files follow (N = 3, P=1): 

```
[INFO] [1653996446.351544]: NEXT GARBAGE AT CELL: (2, 1)
[INFO] [1653996446.353613]: set_direction service starting...
[INFO] [1653996446.355739]: load_garbage service starting....
[INFO] [1653996447.349034]: NEXT GARBAGE AT CELL: (2, 1)
[INFO] [1653996447.351287]: CURRENT ROBOT POSITION: (1, 0)
[INFO] [1653996447.351367]: /robot_node: pointing to cell ((2, 1))
[INFO] [1653996447.354243]: /robot_node: currently at cell ((1, 0))
[INFO] [1653996447.354703]: /robot_node: heading to EAST
[INFO] [1653996447.356531]: /robot_node: heading to EAST
[INFO] [1653996447.366589]: EAST WAS ALREADY SET AS THE CURRENT DIRECTION.
[INFO] [1653996447.367616]: EAST WAS ALREADY SET AS THE CURRENT DIRECTION.
[INFO] [1653996448.349059]: CURRENT ROBOT POSITION: (2, 0)
[INFO] [1653996448.351538]: /robot_node: currently at cell ((2, 0))
[INFO] [1653996448.353558]: /robot_node: heading to NORTH
[INFO] [1653996448.359542]: TRYING TO CHANGE ROBOT DIRECTION FROM  EAST to NORTH.
[INFO] [1653996448.361477]: ( DIRECTION SUCCESSFULLY CHANGED TO NORTH!
[INFO] [1653996449.348982]: CURRENT ROBOT POSITION: (2, 1)
[INFO] [1653996449.351247]: /robot_node: currently at cell ((2, 1))
[INFO] [1653996449.353147]: /robot_node asking to pick up garbage from cell(2, 1)
[INFO] [1653996449.359270]: TRYING TO PIK UP GARBAGE FROM CELL:  2,1
[INFO] [1653996449.361369]: CELL: (2, 1) - GARBAGE CORRECTLY COLLECTED!
[INFO] [1653996449.363524]: |----------------------------------------------------------|
[INFO] [1653996449.365850]: |                                                          |
[INFO] [1653996449.367987]: | CONGRATULATIONS, YOUR EFFORT MADE WORLD A CLEANER PLACE! |
[INFO] [1653996449.369933]: |                                                          |
[INFO] [1653996449.371280]: |----------------------------------------------------------|
```


## Two lines about the architecture

The system consists in two nodes: 

- /robot
- /world

The world node controls the environment (and actually moves the robot) while the robot node contains the logic behind the "decisions" taken. 

Robot is supposed to move in a 2D N x N grid, at the speed of one cell at each second, in one of four possible directions (N,S,W,E), which the robot node can require to change by a service. 

Coordinates are expressed with a two integer touple `(x,y)` ranged between 0 and N -1. 
Cell (0,0) corresponds to the lowest cell on the left side of the grid. 
Directions are expressed with a `(dx,dy)` touple with one of the two values being 0 and the other being `{-1, 1}`. 

For instance direction (-1, 0) would be interpeted as "Horizontally move to the left, or to West", while (0,1) as "Go up or to North". 

Crashes towards walls (the side of the grid) can't happen, due to the implementation of a kind of `"security direction inverter"` changing sign of the non-zero direction coordinate in case of potential crash.  

The robot position is updated by the `/world node` at each second, being published on the `global_position` topic. 

World node also publishes the coordinates of the next piece of garbage to be picked on the `current_destination` topic. 

Both topics are subscribed by the robot node. When the cell containg garbage is reached, another service `load_garbage`, is called, in order to effectively clean the cell.
Once the request is served by the world node, a new garbage cell is randomly chosen and published, so that the loop can contiune, up to P times. 

Values for N and P are setted in the `/config/grid.yaml` file. They can safely be modified for further simulations. 
