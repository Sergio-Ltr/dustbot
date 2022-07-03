#!/usr/bin/python3
import os,sys
import rospy

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from src.utils import cardinal_to_dir

from dustbot.msg import Position
from dustbot.srv import SetDirection, LoadGarbage


class Robot: 
    def __init__(self, initial_cell = [0,0], initial_dir = 'EAST', initial_dest=[0,0]): 
        self.pos = initial_cell
        self.dir = initial_dir
        self.dest = initial_dest


    # Call the set direction service in order to change the current moving direction. 
    def set_direction_client(self, dir): 
        rospy.loginfo(f"{rospy.get_caller_id()}: heading to {dir}")
        rospy.wait_for_service("set_direction")

        dir = cardinal_to_dir(dir)

        try:
            set_direction = rospy.ServiceProxy("set_direction", SetDirection)
            resp = set_direction(x=dir[0], y=dir[1])
            
            if resp.outcome: 
                self.dir = dir

            return resp.outcome 

        except rospy.ServiceException as _: 
            rospy.loginfo(f"{rospy.get_caller_id()} Call to set_direction interrupted")


    # Call the load garbage service in order to actually clean the current cell. 
    def load_garbage_client(self):
        rospy.loginfo(f"{rospy.get_caller_id()} asking to pick up garbage.")
        rospy.wait_for_service("load_garbage")

        try:
            load_garbage = rospy.ServiceProxy("load_garbage", LoadGarbage)
            resp = load_garbage()

            return resp.outcome 

        except rospy.ServiceException as _: 
            rospy.loginfo(f"{rospy.get_caller_id()} Call to load_garbage interrupted")


    # Decide next action: in which direction to go or to load garbage.
    def adjustTrajectory(self): 

        # Get closer to the garbagre horizontally, if not aligned
        if self.dest[0] > self.pos[0]: 
            self.set_direction_client("EAST")
        elif self.dest[0] < self.pos[0]: 
            self.set_direction_client("WEST")

        # Get closer to the garbagre vertically, if not aligned
        elif self.dest[1] > self.pos[1]: 
            self.set_direction_client("NORTH")
        elif self.dest[1] < self.pos[1]: 
            self.set_direction_client("SOUTH")

        #Otherwise it's time to load garbage!
        else :
            self.load_garbage_client()

    
    # Update the current position according to what published on the world topic.
    def pos_callback(self):
        def callback(msg):
            pos = [msg.x, msg.y]
            self.pos = pos

            rospy.loginfo(f"{rospy.get_caller_id()}: currently in cell {pos}")

            self.adjustTrajectory()
        return callback


    # Update the current destination when a new one is published
    def dest_callback(self): 
        def callback(msg):
            dest = [msg.x, msg.y]
            self.dest = dest

            rospy.loginfo(f"{rospy.get_caller_id()}: pointing to cell {dest}")

            self.adjustTrajectory()
        return callback

    
    def begin(self): 
        rospy.init_node(f"robot")

        rospy.Subscriber("global_position", Position, self.pos_callback())
        rospy.Subscriber("current_destination", Position, self.dest_callback())

        rospy.spin()


# Main execution flow of the "robot" node. 
if __name__ == "__main__":
    try:
        Robot().begin()
    except rospy.ROSInterruptException:
        pass
