#!/usr/bin/python3
import rospy

from src.utils import dir_to_cardinal, cardinal_to_dir

from dustbot.msg import Position
from dustbot.srv import SetDirection, LoadGarbage

# Call the set direction service in order to change the current moving direction. 
def set_direction_client(dir): 
    rospy.loginfo(f"{rospy.get_caller_id()}: heading to {dir}")
    rospy.wait_for_service("set_direction")

    try:
        set_direction = rospy.ServiceProxy("set_direction", SetDirection)
        resp = set_direction(dir)
        
        return resp.outcome 

    except rospy.ServiceException as _: 
        rospy.loginfo(f"{rospy.get_caller_id()} Call to set_direction interrupted")


# Call the load garbage service in order to actually clean the current cell. 
def load_garbage_client():
    rospy.loginfo(f"{rospy.get_caller_id()} asking to pick up garbage.")
    rospy.wait_for_service("load_garbage")

    try:
        load_garbage = rospy.ServiceProxy("load_garbage", LoadGarbage)
        resp = load_garbage()

        return resp.outcome 

    except rospy.ServiceException as _: 
        rospy.loginfo(f"{rospy.get_caller_id()} Call to load_garbage interrupted")

# Update the current position according to what published on the world topic.
# Here we should be decide if change direction or keep on ahed. 
def pos_callback(msg, robot):
    rospy.loginfo(f"{rospy.get_caller_id()}: currently in cell ({msg})")
    robot.setPos(msg)

# Update the current destination once the previous one has been "cleaned", according to 
# what published on the world topic. Here we should plan the (best) path to follow.  
def dest_callback(msg, robot): 
    rospy.loginfo(f"{rospy.get_caller_id()}: pointing to cell ({msg})")
    robot.set(msg)

class Robot: 
    def __init__(self, name, initial_cell = [0,0], initial_dir = 'E', initial_dest=[0,0]): 
        self.name = name
        self.pos = initial_cell
        self.dir = initial_dir
        self.dest = initial_dest
        self.history = []

    def clean(self): 
        rospy.init_node(f"robot_{self.name}")

        rospy.Subscriber("global_position", Position, pos_callback, callback_args=self)
        rospy.Subscriber("current_destination", Position, dest_callback, callback_args=self)

        rospy.spin()
        return 

    def changeDirection(self, dir):
        outcome = set_direction_client(dir)
        if outcome:
            self.dir = dir

    def loadGarbage(self):
        outcome = load_garbage_client()
        if outcome:
            self.history.append((self.pos, "L"))

    def setPos(self, pos):
        self.pos = pos
        self.history.append((self.pos, "M"))
        self.adjust_trajectory()

    def setDest(self, dest): 
        self.dest = dest
        self.adjust_trajectory()


    # Decide next action: in which direction to go or to load garbage.
    def adjustTrajecotry(self): 

        # Get closer to the garbagre horizontally, if not aligned
        if self.dest[0] > self.pos[0]: 
            self.set_direction("E")
        elif self.dest[0] < self.pos[0]: 
            self.set_direction("W")

        # Get closer to the garbagre vertically, if not aligned
        elif self.dest[1] > self.pos[1]: 
            self.set_direction("N")
        elif self.dest[1] < self.pos[1]: 
            self.set_direction("S")

        #Otherwise it's time to load garbage!
        else :
            self.load_garbage_client()


# Main execution flow of the "robot" node. 
if __name__ == "__main__":
    try:
        Robot("C1P8").clean()
    except rospy.ROSInterruptException:
        pass
