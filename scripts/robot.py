#!/usr/bin/python3
from cmath import inf
import rospy


from dustbot.msg import Position
from dustbot.srv import SetDirection, LoadGarbage


class Robot: 
    def __init__(self, name, initial_cell = [0,0], initial_dir = 'EAST', initial_dest=[inf,inf]): 
        self.name = name

        self.pos = initial_cell
        self.dir = initial_dir
        self.dest = initial_dest


    # Call the set direction service in order to change the current moving direction. 
    def set_direction(self, dir): 
        rospy.loginfo(f"{rospy.get_caller_id()}: heading to {dir}")
        rospy.wait_for_service("set_direction")

        try:
            resp = self.set_direction_client(dir)
            
            if resp.outcome: 
                self.dir = dir
                rospy.loginfo(f"{rospy.get_caller_id()} direction successfully changed.")

        except rospy.ServiceException as _: 
            rospy.logwarn(f"{rospy.get_caller_id()} Call to set_direction interrupted")


    # Call the load garbage service in order to actually clean the current cell. 
    def load_garbage(self):
        rospy.loginfo(f"{rospy.get_caller_id()} asking to pick up garbage.")
        rospy.wait_for_service("load_garbage")

        try:
            resp =  self.load_garbage_client()

            if resp.outcome:
                rospy.loginfo(f"{rospy.get_caller_id()} garbage successfully loaded.")

            if resp.is_last: 
                rospy.loginfo(f"{rospy.get_caller_id()} it's time to stop.")
                rospy.signal_shutdown("Task is ended")

        except rospy.ServiceException as _: 
            rospy.logwarn(f"{rospy.get_caller_id()} Call to load_garbage interrupted")


    # Decide next action: in which direction to go or to load garbage.
    def adjustTrajectory(self): 
        # Get closer to the garbagre horizontally, if not aligned
        if self.dest[0] > self.pos[0]: 
            self.set_direction("EAST")

        elif self.dest[0] < self.pos[0]: 
            self.set_direction("WEST")

        # Get closer to the garbagre vertically, if not aligned
        elif self.dest[1] > self.pos[1]: 
            self.set_direction("NORTH")
        elif self.dest[1] < self.pos[1]: 
            self.set_direction("SOUTH")

        #Otherwise it's time to load garbage!
        else :
            self.load_garbage()

    
    # Update the current position according to what published on the world topic.
    def pos_callback(self, msg):
        self.pos = msg.pos

        rospy.loginfo(f"{rospy.get_caller_id()}: currently in cell {self.pos}")

        self.adjustTrajectory()


    # Update the current destination when a new one is published
    def dest_callback(self, msg):
        self.dest = msg.pos

        rospy.loginfo(f"{rospy.get_caller_id()}: pointing to cell {self.dest}")

        self.adjustTrajectory()


    def begin(self): 
        rospy.init_node(self.name)

        self.set_direction_client = rospy.ServiceProxy("set_direction", SetDirection)
        self.load_garbage_client = rospy.ServiceProxy("load_garbage", LoadGarbage)

        rospy.Subscriber("current_destination", Position, self.dest_callback)
        rospy.Subscriber("global_position", Position, self.pos_callback)

        rospy.spin()


# Main execution flow of the "robot" node. 
if __name__ == "__main__":
    try:
        Robot('robot').begin()
    except rospy.ROSInterruptException:
        pass
