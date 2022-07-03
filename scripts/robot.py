#!/usr/bin/python3
import rospy

from utils import cardinal_to_dir

from dustbot.msg import Position
from dustbot.srv import SetDirection, LoadGarbage


class Robot: 
    def __init__(self, name, initial_cell = [0,0], initial_dir = 'E', initial_dest=[0,0]): 
        self.name = name
        self.pos = initial_cell
        self.dir = initial_dir
        self.dest = initial_dest
        self.picked_garbage = 0
        self.history = []

    def clean(self): 
        rospy.init_node(f"robot_{self.name}")

        rospy.Subscriber("global_position", Position, self.pos_callback())
        rospy.Subscriber("current_destination", Position, self.dest_callback())

        rospy.spin()
        return 


    def changeDirection(self, dir):
        outcome = self.set_direction_client(dir)
        if outcome:
            self.dir = dir


    def loadGarbage(self):
        outcome = self.load_garbage_client()
        if outcome:
            self.picked_garbage += 1
            self.history.append((self.pos, "L"))


    def setPos(self, pos):
        self.pos = pos
        self.history.append((self.pos, "M"))
        self.adjustTrajectory()


    def setDest(self, dest): 
        self.dest = dest
        self.adjustTrajectory()


    # Decide next action: in which direction to go or to load garbage.
    def adjustTrajectory(self): 

        # Get closer to the garbagre horizontally, if not aligned
        if self.dest[0] > self.pos[0]: 
            self.changeDirection("E")
        elif self.dest[0] < self.pos[0]: 
            self.changeDirection("W")

        # Get closer to the garbagre vertically, if not aligned
        elif self.dest[1] > self.pos[1]: 
            self.changeDirection("N")
        elif self.dest[1] < self.pos[1]: 
            self.changeDirection("S")

        #Otherwise it's time to load garbage!
        else :
            self.loadGarbage()

    # Call the set direction service in order to change the current moving direction. 
    def set_direction_client(self, dir): 
        rospy.loginfo(f"{rospy.get_caller_id()}: heading to {dir}")
        rospy.wait_for_service("set_direction")

        dir = cardinal_to_dir(dir)

        try:
            set_direction = rospy.ServiceProxy("set_direction", SetDirection)
            resp = set_direction(x=dir[0], y=dir[1])
            
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

    # Update the current position according to what published on the world topic.
    def pos_callback(self):
        def callback(msg):
            rospy.loginfo(f"{rospy.get_caller_id()}: currently in cell ({msg})")
            self.setPos((msg.x, msg.y))
        return callback


    # Update the current destination when a new one is published
    def dest_callback(self): 
        def callback(msg):
            rospy.loginfo(f"{rospy.get_caller_id()}: pointing to cell ({msg})")
            self.setDest((msg.x, msg.y))
        return callback


# Main execution flow of the "robot" node. 
if __name__ == "__main__":
    try:
        Robot("C1P8").clean()
    except rospy.ROSInterruptException:
        pass
