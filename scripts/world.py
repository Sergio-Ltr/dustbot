#!/usr/bin/python3

import rospy
import random

from src.utils import dir_to_cardinal

from dustbot.msg import Position

from dustbot.srv import SetDirection, SetDirectionResponse
from dustbot.srv import LoadGarbage, LoadGarbageResponse


P = rospy.get_param('P') #Required pieces of garbage to be picked. 
N = rospy.get_param('N') #Number of cells per row/column.

dir = (1,0) #Assume robot would start moving to the right. 

picked_garbage = 0 #Counter of the picked garbage pieces. 

robot_position = (0,0) #Initial position is the bottom-left corner. 

next_garbage = (random.randint(0, N-1), random.randint(0, N-1)) #Garbage cells are random cells.



# Return four boolean, as the availability of respectively "UP - DOWN - RIGHT - LEFT" directions.
def available_movements(): 

    global N, robot_position, obstacles

    return [      
        robot_position[1] < N - 1, # Can go up?
        robot_position[1] > 0,     # Can go down?
        robot_position[0] < N - 1, # Can go right?
        robot_position[0] > 0      # Can go left?
    ]

# Check if a certain direction change is feasible 
# am is a four elements returned by the available_movements function
def check_blocks(am, dir, single_answer=False): 
    blocked_x = (dir[0] == 1 and not am[2]) or (dir[0] == -1 and not am[3])
    blocked_y = (dir[1] == 1 and not am[0]) or (dir[1] == -1 and not am[1])  

    return blocked_x and blocked_y if single_answer else (blocked_x, blocked_y)

# Publish on the "current_destination" topic the coordinates of the next cell to  
# pick up grbage from.
# Should be called at the beginning of the node execution and when the client notify the 
# "cleance" of the last provided destination.
def publish_next_destination(next_dest):
    pub = rospy.Publisher("current_destination", Position)

    if not rospy.is_shutdown(): 
        rospy.loginfo(f"NEXT GARBAGE AT CELL: {next_dest}")
    
        dest = Position( x = next_dest[0], y = next_dest[1])
        pub.publish(dest)


# Publish on the "global_position" topic the coordinates of the cell where dustbot currently is. 
# Should be called for initialization and whenever the robot moves (in order to update). 
def publish_robot_position(pos):
    pub = rospy.Publisher("global_position", Position)

    if not rospy.is_shutdown(): 
        rospy.loginfo(f"CURRENT ROBOT POSITION: {pos}")

        r_pos = Position(x = pos[0], y = pos[1]) 
        pub.publish(r_pos)

# Contains the so called "Business logic" for the load_garbage call.
def load_garbage_handler(req):
    global picked_garbage, next_garbage, robot_position

    rospy.loginfo(f"TRYING TO PIK UP GARBAGE FROM CELL:  {req.x},{req.y}")
    
    #Here we should check rif there is garbage in the current cell.
    allowed = next_garbage[0] == req.x == robot_position[0] and next_garbage[1] == req.y == robot_position[1]

    if allowed: 
        picked_garbage += 1
        rospy.loginfo(f"CELL: {next_garbage} - GARBAGE CORRECTLY COLLECTED!")
    else: 
        rospy.loginfo(f"CELL: {next_garbage} - UNABLE TO COLLECTED ANY GARBAGE!")
        return LoadGarbageResponse(outcome=False)

    #If it wasn't the last garbage you are supposed to collect, get the next destination
    if picked_garbage < P: 
        next_garbage = (random.randint(0,N-1), random.randint(0,N-1))
        publish_next_destination(next_garbage)
    else: 
        #When entering this block, after the celebrating logs, program should stop.
        rospy.loginfo("|----------------------------------------------------------|")
        rospy.loginfo("|                                                          |")
        rospy.loginfo("| CONGRATULATIONS, YOUR EFFORT MADE WORLD A CLEANER PLACE! |")
        rospy.loginfo("|                                                          |")
        rospy.loginfo("|----------------------------------------------------------|")

    return LoadGarbageResponse(outcome=allowed)


# Initialize the set direction server, in order to be able to actually pick up garbage. 
def load_garbage_server():
    rospy.Service("load_garbage", LoadGarbage, load_garbage_handler)
    rospy.loginfo("load_garbage service starting....")

# Contains the so called "Business logic" for the set direction call. 
def set_direction_handler(req):
    #Get globally set direction
    global dir 

    #This block is present as a defense, but client should never send a request like this!
    if dir[0]== req.x and dir[1] == req.y: 
        rospy.loginfo(f"{dir_to_cardinal(dir)} WAS ALREADY SET AS THE CURRENT DIRECTION.")
        return SetDirectionResponse(outcome=False)

    rospy.loginfo(f"TRYING TO CHANGE ROBOT DIRECTION FROM  {dir_to_cardinal(dir)} to {dir_to_cardinal((req.x, req.y))}.")

    #Here we should check robot would not crash along some "wall".
    am = available_movements()
    
    allowed = not check_blocks(am, (req.x, req.y), single_answer = True)

    if allowed:
        dir = (req.x, req.y)
        rospy.loginfo(f"( DIRECTION SUCCESSFULLY CHANGED TO {dir_to_cardinal(dir)}!")
    else: 
        rospy.loginfo(f"UNABLE TO CHANGE DIRECTION TO {dir_to_cardinal((req.x, req.y))} - WALL DETECTED!")

    return SetDirectionResponse(outcome=allowed)


# Initialize the set direction server, in order to be able to actually move the robot. 
def set_direction_server():
    rospy.Service("set_direction", SetDirection, set_direction_handler)
    rospy.loginfo("set_direction service starting...")


# Initialize the world node, starting the two servers. 
def world(): 
    rospy.init_node("world")

    global N, P, dir, picked_garbage, robot_position, next_garbage

    rate = rospy.Rate(1) # 1 Hz
    # Do stuff, maybe in a while loop

    publish_robot_position(robot_position)
    publish_next_destination(next_garbage)
    
    set_direction_server()
    load_garbage_server()
    
    rate.sleep() # Sleeps for 1/rate sec
    
    garbage_noticed = False

    # Repat at each "rate"
    while picked_garbage < P and not rospy.is_shutdown(): 
        if not garbage_noticed: 
            publish_next_destination(next_garbage)
            garbage_noticed = True

        # Let's move the robot in the setted direction
       

        am = available_movements()

        if picked_garbage == P:
            return

        # Security direction changer, to avoid wall crash...
        blocked_x, blocked_y = check_blocks(am, dir)
        
        if blocked_x: 
            dir = (-dir[0], 0)

        if blocked_y: 
            dir = (0, -dir[1])

        # Actually make the robot move
        robot_position = (robot_position[0] + dir[0], robot_position[1] + dir[1])
        publish_robot_position(robot_position)
    
        rate.sleep() # Sleeps for 1/rate sec   


# Main execution flow of the "world" node. 
if __name__ == "__main__":
    try: 
        world()
    except rospy.ROSInterruptException:
        pass


