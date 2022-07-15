#!/usr/bin/python3
import rospy
import random

from dustbot.utils import cardinal_to_dir, final_log

from dustbot.msg import Position

from dustbot.srv import SetDirection, SetDirectionResponse
from dustbot.srv import LoadGarbage, LoadGarbageResponse


class World(): 
    def __init__(self, name, rate, P, N, initial_cell = [0,0]):
        self.name = name
        self.rate = rate

        self.P = P
        self.N = N

        self.robot_dir = [0, 0]
        self.robot_pos = initial_cell

        self.picked_garbage = 0
        self.garbage_position =  None
        
        self.is_garbage_changed = True

        self.pos_topic = rospy.Publisher("global_position", Position, queue_size=1)
        self.dest_topic = rospy.Publisher("current_destination", Position, queue_size=1)


    # Return four boolean, as the availability of respectively "RIGHT - LEFT - UP - DOWN  RIGHT - LEFT" directions.
    def available_movements(self): 
        N = self.N
        robot_pos = self.robot_pos

        return [
            robot_pos[0] < N - 1, # Can go right?
            robot_pos[0] > 0,     # Can go left?      
            robot_pos[1] < N - 1, # Can go up?
            robot_pos[1] > 0      # Can go down?
        ]


    # Check if a certain direction change is feasible 
    def check_blocks(self, dir): 
        am = self.available_movements()

        blocks = [
            (dir[0] == 1 and not am[0]),
            (dir[0] == -1 and not am[1]),
            (dir[1] == 1 and not am[2]),
            (dir[1] == -1 and not am[3])
        ]
        
        blocked_x = blocks[0] or blocks[1] 
        blocked_y = blocks[2] or blocks[3]
       
        return blocked_x or blocked_y


    # Generate a random coordinates pair for the next garbage and publish it.
    def get_next_garbage(self): 
        self.garbage_position = (random.randint(0, self.N-1), random.randint(0, self.N-1))
    

    def move_robot(self):
        dir = self.robot_dir
        if not self.check_blocks(dir):
            self.robot_pos[0] += dir[0]
            self.robot_pos[1] += dir[1]
        

    # Publish on the "global_position" topic the coordinates of the cell where dustbot currently is. 
    def publish_robot_position(self, pos):
        if not rospy.is_shutdown(): 
            rospy.loginfo(f"CURRENT ROBOT POSITION: {pos}")
            self.pos_topic.publish(Position(pos))


    # Publish on the "current_destination" topic the coordinates of the grbage.
    def publish_next_destination(self):
        dest = self.garbage_position
        if not rospy.is_shutdown(): 
            rospy.loginfo(f"NEXT GARBAGE AT CELL: {dest}")
            self.dest_topic.publish(Position(dest))


    # Builds and return a handler for the load garbage service. 
    def load_garbage_handler(self, _):
        pos = self.robot_pos
        garbage = self.garbage_position

        rospy.loginfo(f"TRYING TO PIK UP GARBAGE FROM CELL:  {pos}")
        
        #Here we should check if there is garbage in the current cell.
        allowed = garbage[0] == pos[0] and garbage[1] == pos[1]

        if allowed: 
            self.picked_garbage += 1
            rospy.loginfo(f"CELL: {garbage} - GARBAGE CORRECTLY COLLECTED!")
        else: 
            rospy.logwarn(f"CELL: {garbage} - UNABLE TO COLLECTED ANY GARBAGE!")
            return LoadGarbageResponse(False, False)

        #If it wasn't the last garbage you are supposed to collect, get the next destination
        if self.picked_garbage < self.P: 
            self.get_next_garbage()
            self.is_garbage_changed = True
            #self.publish_next_destination()
        else: 
            #When entering this block, after the celebrating logs, program should stop.
            for log in final_log(): 
                rospy.loginfo(log)

        return LoadGarbageResponse(True, self.picked_garbage >= P)


    # Builds and return a handler for the set direction service. 
    def set_direction_handler(self, req): 
        current_dir = self.robot_dir
        new_dir = cardinal_to_dir(req.dir)

        #This block is present as a defense, but client should never send a request like this!
        if current_dir != None and current_dir == new_dir: 
            rospy.loginfo(f"{req.dir} WAS ALREADY SET AS THE CURRENT DIRECTION.")
            return SetDirectionResponse(False)
        else: 
            rospy.loginfo(f"TRYING TO CHANGE ROBOT DIRECTION TO {req.dir}.")

        #Here we should check robot would not crash along some "wall".
        allowed = not self.check_blocks(new_dir)

        if allowed:
            rospy.loginfo(f" DIRECTION SUCCESSFULLY CHANGED TO {req.dir}!")
            self.robot_dir = new_dir
            return SetDirectionResponse(True)
        else: 
            rospy.logwarn(f"UNABLE TO CHANGE DIRECTION TO {req.dir} - WALL DETECTED!")
            return SetDirectionResponse(False)


    # Start world handling loop. 
    def begin(self): 
        rospy.init_node(self.name)
        rate = rospy.Rate(self.rate)

        rospy.Service("load_garbage", LoadGarbage, self.load_garbage_handler)
        rospy.loginfo("load_garbage service starting....")

        rospy.Service("set_direction", SetDirection, self.set_direction_handler)
        rospy.loginfo("set_direction service starting...")

        self.get_next_garbage() #Generate the random coordinates of the next garbage-cell. 

        while self.picked_garbage < P and not rospy.is_shutdown(): 
            # Once per second, make the robot move
            self.move_robot()
            self.publish_robot_position(self.robot_pos)
            if self.is_garbage_changed or self.robot_dir == [0,0]: 
                self.publish_next_destination()
                self.is_garbage_changed = False

            rate.sleep()


# Main execution flow of the "world" node. 
if __name__ == "__main__":
    try: 
        P = rospy.get_param('P') #Required pieces of garbage to be picked. 
        N = rospy.get_param('N') #Number of cells per row/column.
    
        World( "world", 1, P, N).begin()
    except rospy.ROSInterruptException:
        pass
