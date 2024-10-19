
"""
1 get waypoints
2 get max_loops
3 use max_loops in a loop along with a counter to loop through every loop
4 increiment the counter by 1 to increase the loop number, add half the plow width every loop 
5 add extra logic at the end of the cycle 
"""

FIRST_CORNER = (0, 0)
SECOND_CORNER = (100, 200)
SNOWPLOW_WIDTH = 4

"""finds the exact width of the rectangle"""
def get_width(first_corner, second_corner, Spwidth):
    Width = second_corner[1] - first_corner[1]
    return(round(Width/Spwidth)) # number of strips that the plow must travel

"""this function may still need some work"""
"""finds the exact center of the rectangle"""
def get_center(waypoints):
    center = len(waypoints)/2
    return waypoints[center] # returns the tuple that is at the center of the waypoints list

"""function to build the spiral"""
#TODO: modify so that it makes paths with the proper width for the parking lot
def spiral(X, Y, first_corner, center):
    x = y = first_corner[0] ,first_corner[1]
    dx = center[0]
    dy = center[1]
    for i in range(max(X, Y)**2):
        if (-X/2 < x <= X/2) and (-Y/2 < y <= Y/2):
            print (x, y)
            # DO STUFF...
        if x == y or (x < 0 and x == -y) or (x > 0 and x == 1-y):
            dx, dy = -dy, dx
        x, y = x+dx, y+dy


"""builds the actual path for the plow to run """
def make_path(second_corner, waypoints, strips):
    BottomEdge = second_corner[0]
    RightEdge = second_corner[1]
    path = [round(Spwidth/2), 0] # starting position would be in the center of the plow
    for i in waypoints:
        while i <= second_corner[1]:
            path.append([])



"""Gemerates waypoints based on a rectangle defined by its corners."""
def generate_waypoints(first_corner, second_corner):
    waypoints = []
    for i in range(second_corner[0] + 1):
        for j in range(second_corner[1] + 1): # plus one so we get 0 to the very back of the rectangle
            waypoints.append((i, j))
    return waypoints


if __name__ == "__main__":
    first_corner=FIRST_CORNER 
    second_corner=SECOND_CORNER
    Spwidth=SNOWPLOW_WIDTH
    waypoints = generate_waypoints(first_corner, second_corner)
    strips = get_width(first_corner, second_corner, Spwidth)
    center = get_center(waypoints)
    path = spiral(second_corner[0], second_corner[1]. first_corner, center)
    



#"""Generates every point in the strip that the vehicle will have to travel"""
#def generate_strip_lists(second_corner, strips, Spwidth):
#    stripstart = Spwidth
#    striplist = []
#    
#    for i in range(strips):
#        tmplist = []
# 
#        for j in range(second_corner[1] + 1):
#            tmplist.append([stripstart, j])
#        
#        striplist.append([tmplist])
#        stripstart += Spwidth  # moves the x coord up to the next strip 
#    
#    return striplist

