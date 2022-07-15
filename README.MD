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

## Exercise Specification

Dustbot 2.0,is a mobile robot, navigates a 2-dimensional NxN grid along the four directions (N-S-E-W) with a speed of one `grid-cell/s`.

Its global position in the grid in X,Y coordinates are published on the `/dustbot/global_position` ROS Topic.

The direction in which the robot moves can be altered using the custom ROS service `/dustbot/set_direction`.

Dustbot needs to pick up garbage at specific world coordinate which, are available on the `/current_destination` topic.

Monitoring it, Dustbot can figure out towards which direction to move.

Once reached its destination, Dustbot has to initiate the garbage pick up calling the `/load_garbage service` (picking up will succed only at `current _ destination` ).

Once the garbage has been loaded, a new random destination gets for Dustbot to move on towards. After P pick-ups the program ends.

The Size of the grid (N) and number of pick-ups (P) should be loaded on startup using rosparam.

Use a launch file to run multiple nodes and the N and P parameters via rosparam.

## Architecture and Files

The system consists in two nodes: 

- `/robot_node` implemented in the `robot.py` script.
- `/world_node` implemented in the `world.py` script.

- An additional python code, containing some utility functions, named `utils.py`, can be found in the `src` folder and is compiled as an external library following the logic of `catkin`. http://docs.ros.org/en/jade/api/catkin/html/howto/format2/installing_python.html

- Services are implement according to specification and can be found in the proprer `srv` file: `SetDirection.srv` and `LoadGarbage.srv`. 

- One single message is used both for `robot_position` and `current_destination` ROS Topic, and is declared in the  `Position.srv` file.  

- Values for N and P are setted in the `/config/grid.yaml` file. They can safely be modified for further simulations. 


## Architecture and lifecycle

The world node controls the environment (and actually moves the robot) while the robot node contains the logic behind the directions taken. 

Robot is supposed to move in a 2D N x N grid, at the speed of one cell at each second, in one of four possible directions (N,S,W,E), which the robot node can require to change by a service. 

Robot position is updated by the `/world node` at each second, being published on the `/global_position` topic. 

Position coordinates are expressed with a two integers ranged between 0 and N -1 collected inside a (fixed length) vector `[x, y]` both codewise ad within the corresponding `Position` ROS message. Cell `[0,0]` corresponds to the lowest cell on the left side of the grid. 

Destinations are published, as `Position` ROS message as well, on the `current_destination` ROS Topic at the launch of the `world_node` and whenever a piece of garbage is collected. In this implmentation destinations are randomly generated couple of  coordinates within the grid. 

When robot receives a new destination, a new direction is computed according to its current position by the `adjust_trajectory` function, which is also called each time robot moves, or in other terms whenever its position is published. 

Once the new direction is computed, `robot_node` sends a request to the `set_direction` Service, which is handled within the `world_node`, uptading the robot direction (if it has changend with respect to the previous one) and returning an boolean outcome value. 

Within the ROS service, directions can only be expressed with the four uppercase cardinal points as uppercase strings.

Within code scripts, like positions, directions are expressed again with a two elements `[dx,dy]` vector, having 0 as one of the two values and the other being only `{-1, 1}`. For example, direction `[-1, 0]` would be interpeted as "Horizontally move to the left, or to West", while `[0, 1]` as "Go up or to North". Such implementation makes robot movement equivalent to an easy `position + direction` sum. 

A function converting cardinal points directions into 2-elements array is implemented in the utils.

A special case [0, 0] direction vector is used to express the initial state of robot, which is stationary. Such a null direction is never set again when movement starts.

Direction changed are supposed to keep robot to the destination cell. Once there, insted of set a new direction, a `robot_node` sends a request to the  'load_garbage' Service, which is handled by the `world_node`, checking if cells actually corresponds and responding again with two boolean values. First one is for the pick up operation outcome, while second one is set to true only after the last garbage has been picked.

In case of a positive pickup, a counter for the picked pieces of garbage is incremented, in order to be compared with the setted P value as entrance condition for the main while loop. 


## Implementative details

Everithing described in the upper paragraph is logged during each execution. 
For the majority of message the `rospy.loginfo` method was used. Logs from the `world_node` are always uppercase, while logs from `robot_node` and start with a preix. Unexpected behaviours are logged using the `rospy.logwarn` method. 

Parameter reading from YAML file can potentially cause runtime errors, and so is handled within a try except block.

Crashes towards walls (the side of the grid) can't happen, due to the implementation of a kind of "security direction checker" which inhibits robot movement in case its next move would cause of potential crash. Such a control is invoked before by the `world_node` before changing robot position.

Asyncronous start of the nodes are handled, in order to obtain the same behaviour indepently from the starting order: 
 - if the `world_node` would start with some delay, `robot_node` would stay waiting for a destination, thansk to the behaviour of `rospy.spin()` method.
 - if the `robot_node` would start with some delay, `world_node` would not actually move the robot  thanks to the alredy mentioned initial direction set to `[0,0]`. As long as this direction does not change, `world_node` will keep publishing, once per secondo, the current destination, along with the robot position, wich will obviously keep being the inital one.  

Node terminations happen when the P-th piece of garbage is collected: 
- `world_node` ends programmatically, exiting from the main while loop.
- `robot_node` gets informed by the response from the `LoadGarbageService`, expressed by its second and last boolean parameter `is_last`: if is value is  true, the node is terminated using the `rospy.signal_shutdown` method. 

Additionally, the `required` flag has been set to true for the `world_node` in the lauch file, so that whenever that node temrinats, all the exectuion is stopped (including the rosmaster, implicitly starded). 
