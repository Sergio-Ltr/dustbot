
#Given a direction expressed as a (0,1) coordinates couples return the corresponding cardinal point string
def dir_to_cardinal(dir): 
    cardinal = "WEST"

    if dir[0]== 0: 
        cardinal = "NORTH" if dir[1] == 1 else "SOUTH"
    elif dir[0] == 1:
        cardinal = "EAST"
    
    return cardinal