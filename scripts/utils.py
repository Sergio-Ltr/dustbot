#Given a direction expressed as a (0,1) coordinates couples return the corresponding cardinal point string
def dir_to_cardinal(dir): 
    cardinal = "W"

    if dir[0]== 0: 
        cardinal = "N" if dir[1] == 1 else "S"
    elif dir[0] == 1:
        cardinal = "E"
    
    return cardinal

def cardinal_to_dir(cardinal): 

    if cardinal == 'N':
        return (0,1)
    elif cardinal == 'S':
        return (0,-1)
    elif cardinal == 'W':
        return (-1,0)
    else: #cardianl = 'E'
        return (1,0)

def final_log(): 
    return [
        "|----------------------------------------------------------|",
        "|                                                          |",
        "| CONGRATULATIONS, YOUR EFFORT MADE WORLD A CLEANER PLACE! |",
        "|                                                          |",
        "|----------------------------------------------------------|"
    ]