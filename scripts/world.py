#!/usr/bin/python3
import os
import rospy
import random

from utils import dir_to_cardinal, final_log

from dustbot.msg import Position

from dustbot.srv import SetDirection, SetDirectionResponse
from dustbot.srv import LoadGarbage, LoadGarbageResponse


class World(): 
    def __init__(self, P, N, initial_cell = [0,0]):
        self.P = P
        self.N = N

        self.posTopic = rospy.Publisher("global_position", Position, queue_size=1)
        self.destTopic = rospy.Publisher("current_destination", Position, queue_size=1)

        self.robotPos = initial_cell
        self.robotDir = None
        self.garbage = None

        self.pickedGarbage = 0


    # Return four boolean, as the availability of respectively "UP - DOWN - RIGHT - LEFT" directions.
    def available_movements(self): 
        N = self.N
        robotPos = self.robotPos

        return [
            robotPos[0] < N - 1, # Can go right?
            robotPos[0] > 0,     # Can go left?      
            robotPos[1] < N - 1, # Can go up?
            robotPos[1] > 0      # Can go down?
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
        self.garbage = (random.randint(0, self.N-1), random.randint(0, self.N-1))
    

    def moveRobot(self):
        dir = self.robotDir
        if not self.check_blocks(dir):
            self.robotPos[0] += dir[0]
            self.robotPos[1] += dir[1]
            self.update_robot_position(self.robotPos)
        

    # Publish on the "global_position" topic the coordinates of the cell where dustbot currently is. 
    def update_robot_position(self, pos):
        if not rospy.is_shutdown(): 
            rospy.loginfo(f"CURRENT ROBOT POSITION: {pos}")
            self.posTopic.publish(Position(x = pos[0], y = pos[1]))


    # Publish on the "current_destination" topic the coordinates of the grbage.
    def update_next_destination(self):
        dest = self.garbage
        if not rospy.is_shutdown(): 
            rospy.loginfo(f"NEXT GARBAGE AT CELL: {dest}")
            self.destTopic.publish(Position( x = dest[0], y = dest[1]))


    # Builds and return a handler for the load garbage service. 
    def load_garbage_handler(self):
        def handler(_): 
            pos = self.robotPos
            garbage = self.garbage

            rospy.loginfo(f"TRYING TO PIK UP GARBAGE FROM CELL:  {pos}")
            
            #Here we should check if there is garbage in the current cell.
            allowed = garbage[0] == pos[0] and garbage[1] == pos[1]

            if allowed: 
                self.pickedGarbage += 1
                rospy.loginfo(f"CELL: {garbage} - GARBAGE CORRECTLY COLLECTED!")
            else: 
                rospy.loginfo(f"CELL: {garbage} - UNABLE TO COLLECTED ANY GARBAGE!")
                return LoadGarbageResponse(False)

            #If it wasn't the last garbage you are supposed to collect, get the next destination
            if self.pickedGarbage < self.P: 
                self.get_next_garbage()
                self.update_next_destination()
            else: 
                #When entering this block, after the celebrating logs, program should stop.
                for log in final_log(): 
                    rospy.loginfo(log)

                os.system("rosnode kill robot_node") #Maybe too naive?

            return LoadGarbageResponse(True)

        return handler


    # Builds and return a handler for the set direction service. 
    def set_direction_handler(self): 
        def handler(req): 
            current_dir = self.robotDir
            new_dir = [req.x, req.y]

            #This block is present as a defense, but client should never send a request like this!
            if current_dir != None and current_dir == new_dir: 
                rospy.loginfo(f"{dir_to_cardinal(new_dir)} WAS ALREADY SET AS THE CURRENT DIRECTION.")
                return SetDirectionResponse(False)
            else: 
                rospy.loginfo(f"TRYING TO CHANGE ROBOT DIRECTION TO {dir_to_cardinal(new_dir)}.")

            #Here we should check robot would not crash along some "wall".
            allowed = not self.check_blocks(new_dir)

            if allowed:
                rospy.loginfo(f"DIRECTION SUCCESSFULLY CHANGED TO {dir_to_cardinal(new_dir)}!")
                self.robotDir = new_dir
                return SetDirectionResponse(True)
            else: 
                rospy.loginfo(f"UNABLE TO CHANGE DIRECTION TO {dir_to_cardinal(new_dir)} - WALL DETECTED!")
                return SetDirectionResponse(False)

        return handler


    # Start world handling loop. 
    def begin(self): 
        rospy.init_node("world")
        rate = rospy.Rate(1)

        rospy.Service("load_garbage", LoadGarbage, self.load_garbage_handler())
        rospy.loginfo("load_garbage service starting....")

        rospy.Service("set_direction", SetDirection, self.set_direction_handler())
        rospy.loginfo("set_direction service starting...")

        self.get_next_garbage()
        self.update_next_destination()

        rate.sleep()
        rate.sleep()

        while self.pickedGarbage < P and not rospy.is_shutdown(): 
            # Once per second, make the robot move
            self.moveRobot()

            rate.sleep()


# Main execution flow of the "world" node. 
if __name__ == "__main__":
    try: 
        P = rospy.get_param('P') #Required pieces of garbage to be picked. 
        N = rospy.get_param('N') #Number of cells per row/column.

        World(P, N).begin()
    except rospy.ROSInterruptException:
        pass
