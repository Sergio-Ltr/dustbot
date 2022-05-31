#!/usr/bin/python3

import rospy

from utils import dir_to_cardinal

from dustbot.msg import Position
from dustbot.srv import SetDirection, LoadGarbage

current_pos = (0,0) # Robot will always start in the top left cell. 
current_dest = (0,0) # First destination will be read when published

# Update the current position according to what published on the world topic.
# Here we should be decide if change direction or keep on ahed. 
def pos_callback(msg):
    global current_pos
    current_pos = (msg.x, msg.y)
    rospy.loginfo(f"{rospy.get_caller_id()}: currently at cell ({current_pos})")
    adjust_trajectory()

# Update the current destination once the previous one has been "cleaned", according to 
# what published on the world topic. Here we should plan the (best) path to follow.  
def dest_callback(msg): 
    global current_dest
    current_dest = (msg.x, msg.y)
    rospy.loginfo(f"{rospy.get_caller_id()}: pointing to cell ({current_dest})")
    adjust_trajectory()

# Decide next action: in which direction to go or to load garbage.
def adjust_trajectory(): 
    global current_dest, current_pos

    x_r, y_r = current_pos
    x_g, y_g = current_dest
    

    # Get closer to the garbagre horizontally, if not aligned
    if x_g != x_r: 
        dir = ((1,0) if x_g > x_r else (-1, 0))
        set_direction_client(dir[0], dir[1])
        return

    # Get closer to the garbagre vertically, if not aligned
    if y_g != y_r: 
        dir = ((0,1) if y_g > y_r else (0,-1))
        set_direction_client(dir[0], dir[1])
        return 

    # It's time to load garbage!
    load_garbage_client(current_pos[0] ,current_pos[1])

# Check if a certain direction change is feasible 
def check_blocks(am, dir, single_answer=False): 
    blocked_x = (dir[0] == 1 and not am[2]) or (dir[0] == -1 and not am[3])
    blocked_y = (dir[1] == 1 and not am[0]) or (dir[1] == -1 and not am[1])  

    return blocked_x and blocked_y if single_answer else (blocked_x, blocked_y)


# To be call at the beginning of the execution, in order to create the node and make the
# robot "aware" of its position in  the space. 
# Also the grid dimension should become known, through reading of the rosparams. 
def initializer():
    rospy.init_node("robot")

    rospy.Subscriber("global_position", Position, pos_callback)
    rospy.Subscriber("current_destination", Position, dest_callback)

    rospy.spin()


# Call the set direction service in order to change the current moving direction. 
def set_direction_client(x,y): 
    rospy.loginfo(f"{rospy.get_caller_id()}: heading to {dir_to_cardinal((x,y))}")
    rospy.wait_for_service("set_direction")

    try:
        set_direction = rospy.ServiceProxy("set_direction", SetDirection)
        resp = set_direction(x,y)
        
        return resp.outcome 

    except rospy.ServiceException as _: 
        rospy.loginfo(f"{rospy.get_caller_id()} Call to set_direction interrupted")


# Call the load garbage service in order to actually clean the current cell. 
def load_garbage_client(x,y):
    rospy.loginfo(f"{rospy.get_caller_id()} asking to pick up garbage from cell{(x,y)}")
    rospy.wait_for_service("load_garbage")
    try:
        load_garbage = rospy.ServiceProxy("load_garbage", LoadGarbage)
        resp = load_garbage(x,y)

        return resp.outcome 

    except rospy.ServiceException as _: 
        rospy.loginfo(f"{rospy.get_caller_id()} Call to load_garbage interrupted")


# Main execution flow of the "robot" node. 
if __name__ == "__main__":
    
    try:
        initializer()
    except rospy.ROSInterruptException:
        pass
