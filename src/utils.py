import numpy as np  

#Given a direction expressed as a (0,1) coordinates couples return the corresponding cardinal point string
def dir_to_cardinal(dir): 
    cardinal = "WEST"

    if dir[0]== 0: 
        cardinal = "NORTH" if dir[1] == 1 else "SOUTH"
    elif dir[0] == 1:
        cardinal = "EAST"
    
    return cardinal

def cardinal_to_dir(cardinal): 
    dir = [0,0]
    
    if cardinal == 'N':
        dir[1] = 1 
    elif cardinal == 'S':
        dir[1] = -1  
    elif cardinal == 'W':
        dir[0] = -1
    else: 
        dir[0] = 1

    return np.array(cardinal) 