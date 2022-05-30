#!/usr/bin/python3

import rospy
import random

from dustbot.msg import Position, Garbage

from dustbot.srv import SetDirection, SetDirectionResponse
from dustbot.srv import LoadGarbage, LoadGarbageResponse

P = rospy.get_param('P') #Required pieces of garbage to be picked. 

N = rospy.get_param('N') #Number of cells per row/column.

dir = (1,0) #Assume robot would start moving to the right. 

obstacles = [] #Not used, maybe will be implemented in the future. 

picked_garbage = 0 #Counter of the picked garbage pieces. 

robot_position = (0,0) #Initial position is the bottom-left corner. 

next_garbage = (random.randint(0, N-1), random.randint(0, N-1)) #Garbage cells are random cells.

history = [] #Garbage cells are random cells.

# Return four boolean, as the availability of respectively "UP - DOWN - RIGHT - LEFT" directions.
def available_movements(): 

    global N, robot_position, obstacles

    return [      
        robot_position[1] < N - 1, # Can go up?
        robot_position[1] > 0,     # Can go down?
        robot_position[0] < N - 1, # Can go right?
        robot_position[0] > 0      # Can go left?
    ]


# Contains the so called "Business logic" for the load_garbage call.
def load_garbage_handler(req):
    rospy.loginfo(f"Picking up garnage from current cell:  {req.x},{req.y}")
    
    global picked_garbage, next_garbage

    #Here we should check rif there is garbage in the current cell.
    print(next_garbage)
    allowed = next_garbage[0] == req.x and next_garbage[1] == req.y

    if allowed: 
        picked_garbage += 1
        rospy.loginfo(f"Garbage being collected from cell: {next_garbage}")
        
        
    if picked_garbage < P: 
        next_garbage = (random.randint(0,N-1), random.randint(0,N-1))
        publish_next_destination(next_garbage)
    else: 
        rospy.loginfo("|----------------------------------------------------------|")
        rospy.loginfo("|                                                          |")
        rospy.loginfo("| Congratulations, your effort made world a cleaner place! |")
        rospy.loginfo("|                                                          |")
        rospy.loginfo("|----------------------------------------------------------|")

    return LoadGarbageResponse(outcome=allowed)


# Initialize the set direction server, in order to be able to actually pick up garbage. 
def load_garbage_server():
    rospy.Service("load_garbage", LoadGarbage, load_garbage_handler)
    rospy.loginfo("Ready to clean the world :)")


# Contains the so called "Business logic" for the set direction call. 
def set_direction_handler(req):
    rospy.loginfo(f"Changing robot direction into  {req.x},{req.y}")

    global dir 

    #Here we should check robot would not crash along some "wall".
    am = available_movements()
    print("Available sides",am)

    dir = (req.x, req.y)

    allowed = True 
    return SetDirectionResponse(outcome=allowed)


# Initialize the set direction server, in order to be able to actually move the robot. 
def set_direction_server():
    rospy.Service("set_direction", SetDirection, set_direction_handler)
    rospy.loginfo("Ready to change robot direction")


# Publish on the "current_destination" topic the coordinates of the next cell to  
# pick up grbage from.
# Should be called at the beginning of the node execution and when the client notify the 
# "cleance" of the last provided destination.
def publish_next_destination(next_dest):
    pub = rospy.Publisher("current_destination", Position, queue_size=1)

    if not rospy.is_shutdown(): 
        dest_str = f"Next garbage at ({next_dest[0]}, {next_dest[1]})"
        rospy.loginfo(dest_str)
        pos = Garbage( x = next_dest[0], y = next_dest[1])
        pub.publish(pos)


# Publish on the "global_position" topic the coordinates of the cell where dustbot currently is. 
# Should be called for initialization and whenever the robot moves (in order to update). 
def publish_robot_position(pos):
    pub = rospy.Publisher("global_position", Position, queue_size=1)

    if not rospy.is_shutdown(): 
        pos_str = f"Robot currently at position {pos[0]}, {pos[1]}"
        rospy.loginfo(pos_str)
        r_pos = Position(x = pos[0], y = pos[1]) 
        pub.publish(r_pos)

    
# Initialize the world node, starting the two servers. 
def world(): 
    rospy.init_node("world")

    global N, P, dir, picked_garbage, robot_position, next_garbage, history

    rate = rospy.Rate(1) # 1 Hz
    # Do stuff, maybe in a while loop

    publish_robot_position(robot_position)
    publish_next_destination(next_garbage)

    set_direction_server()
    load_garbage_server()
    
    rate.sleep() # Sleeps for 1/rate sec

    # Repat at each "rate"
    while picked_garbage < P: 
        # Let's move the robot in the setted direction
        am = available_movements()

        if picked_garbage == P:
            return

        # Security direction checker
        blocked_x = (dir[0] == 1 and not am[2]) or (dir[0] == -1 and not am[3])
        blocked_y = (dir[1] == 1 and not am[0]) or (dir[1] == -1 and not am[1])  
        
        if blocked_x: 
            dir = (-dir[0], 0)

        if blocked_y: 
            dir = (0, -dir[1])

        # Actually make the robot move
        robot_position = (robot_position[0] + dir[0], robot_position[1] + dir[1])
        publish_robot_position(robot_position)
        # publish_next_destination(next_garbage[0])

        rate.sleep() # Sleeps for 1/rate sec    


# Main execution flow of the "world" node. 
if __name__ == "__main__":
    N = 10 # TODO read from rosparam
    P = 3  # TODO read from rosparam

    try: 
        world()
    except rospy.ROSInterruptException:
        pass


