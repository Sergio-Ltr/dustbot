#Given a direction expressed as a (0,1) coordinates couples return the corresponding cardinal point string
def dir_to_cardinal(dir): 
    cardinal = "WEST"

    if dir[0]== 0: 
        cardinal = "NORTH" if dir[1] == 1 else "SOUTH"
    elif dir[0] == 1:
        cardinal = "EAST"
    
    return cardinal

def cardinal_to_dir(cardinal): 

    if cardinal == 'NORTH':
        return (0,1)
    elif cardinal == 'SOUTH':
        return (0,-1)
    elif cardinal == 'WEST':
        return (-1,0)
    else: #cardianl == 'EAST'
        return (1,0)

def final_log(): 
    return [
        "|----------------------------------------------------------|",
        "|                                                          |",
        "| CONGRATULATIONS, YOUR EFFORT MADE WORLD A CLEANER PLACE! |",
        "|                                                          |",
        "|----------------------------------------------------------|"
    ]