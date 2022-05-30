#!/usr/bin/python3

import rospy

from dustbot.msg import Position
from dustbot.srv import SetDirection, LoadGarbage

current_pos = (0,0)
current_dest = (0,0)

# Update the current position according to what published on the world topic.
# Here we should be decide if change direction or keep on ahed. 
def pos_callback(msg):
    rospy.loginfo(f"{rospy.get_caller_id()}: Robot at cell ({msg.x}, {msg.y})")
    global current_pos
    current_pos = (msg.x, msg.y)
    rospy.loginfo(f"Current position: ({msg.x},{msg.y})")
    adjust_trajectory()


# Update the current destination once the previous one has been "cleaned", according to 
# what published on the world topic. Here we should plan the (best) path to follow.  
def dest_callback(msg): 
    rospy.loginfo(f"{rospy.get_caller_id()}:  Garbage at cell ({msg.x}, {msg.y})")
    global current_dest
    current_dest = (msg.x, msg.y)
    rospy.loginfo(f"Current destination: ({msg.x},{msg.y})")
    adjust_trajectory()

def adjust_trajectory(): 
    global current_dest
    global current_pos

    x_r, y_r = current_pos
    x_g, y_g = current_dest
  
    # Get closer to the garbagre horizontally, if not aligned
    if(x_g != x_r): 
        dir = ((1,0) if x_g > x_r else (-1, 0))
        set_direction_client(dir[0], dir[1])
        return

    # Get closer to the garbagre vertically, if not aligned
    if(y_g != y_r): 
        dir = ((0,1) if y_g > y_r else (0,-1))
        set_direction_client(dir[0], dir[1])
        return 

    # It's time to load garbage!
    rospy.loginfo(f"Time to load trash at {current_pos}")
    load_garbage_client(current_pos[0] ,current_pos[1])


# To be call at the beginning of the execution, in order to create the node and make the
# robot "aware" of its position in  the space. 
# Also the grid dimension should become known, through reading of the rosparams. 
def initializer():
    rospy.init_node("robot")

    rospy.Subscriber("global_position", Position, pos_callback)
    rospy.Subscriber("current_destination", Position, dest_callback)
    
    #@Todo implement the move service.
    rospy.spin()


# Call the set direction service in order to change the current moving direction. 
def set_direction_client(x,y): 
    rospy.loginfo(f"Asking direction change towards ({x}, {y})")
    rospy.wait_for_service("set_direction")
    try:
        set_direction = rospy.ServiceProxy("set_direction", SetDirection)
        resp = set_direction(x,y)

        return resp.outcome 

    except rospy.ServiceException as _: 
        print("Call to set_direction interrupted")


# Call the load garbage service in order to actually clean the current cell. 
def load_garbage_client(x,y):
    rospy.loginfo(f"Asking to load garbage from cell ({x}, {y})")
    rospy.wait_for_service("load_garbage")
    try:
        load_garbage = rospy.ServiceProxy("load_garbage", LoadGarbage)
        resp = load_garbage(x,y)

        return resp.outcome 

    except rospy.ServiceException as _: 
        print("Call to load_garbage interrupted")


# Main execution flow of the "robot" node. 
if __name__ == "__main__":
    
    try:
        initializer()
    except rospy.ROSInterruptException:
        pass
