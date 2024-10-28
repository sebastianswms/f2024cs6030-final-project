"""Gemerates waypoints based on a rectangle defined by its corners."""
def generate_waypoints(second_corner):
    waypoints = []
    for i in range(second_corner[0] + 1):
        for j in range(second_corner[1] + 1): # plus one so we get 0 to the very back of the rectangle
            waypoints.append((i, j))
    return waypoints

"""finds the exact width of the rectangle"""
def get_width(first_corner, second_corner, Spwidth):
    width = second_corner[0] - first_corner[0]
    return(round(width))

"""finds the exact height of the rectangle"""
def get_height(first_corner, second_corner, Spwidth):
    height = second_corner[1] - first_corner[1]
    return(round(height))

"""this function may still need some work"""
"""finds the exact center of the rectangle"""
def get_center(waypoints):
    center = round(len(waypoints)/2)
    return waypoints[center] # returns the tuple that is at the center of the waypoints list

"""function to build the spiral"""
def spiral(TlCorner, Brcorner, rectwidth, rectheight):
    # Initialize current position
    currpoint = TlCorner
    spirallist = []
    
    # Initialize direction offsets (right, down, left, up)
    current_direction = 0  # start moving right

    # Initialize bounds for spiraling
    left_bound = 0
    right_bound = rectwidth
    top_bound = 0
    bottom_bound = rectheight
    
    # Counter for spiral filling
    count = 0  # Count how many steps are taken

    while left_bound <= right_bound and top_bound <= bottom_bound:
        if current_direction == 0:  # Moving right
            for i in range(left_bound, right_bound + 1):
                spirallist.append((TlCorner[0] + i, TlCorner[1] + top_bound))
                count += 1
            top_bound += 1
        elif current_direction == 1:  # Moving down
            for i in range(top_bound, bottom_bound + 1):
                spirallist.append((TlCorner[0] + right_bound, TlCorner[1] + i))
                count += 1
            right_bound -= 1
        elif current_direction == 2:  # Moving left
            for i in range(right_bound, left_bound - 1, -1):
                spirallist.append((TlCorner[0] + i, TlCorner[1] + bottom_bound))
                count += 1
            bottom_bound -= 1
        elif current_direction == 3:  # Moving up
            for i in range(bottom_bound, top_bound - 1, -1):
                spirallist.append((TlCorner[0] + left_bound, TlCorner[1] + i))
                count += 1
            left_bound += 1
        
        # Update direction
        current_direction = (current_direction + 1) % 4
    
    return spirallist

waypoints = generate_waypoints((5, 10))
width = get_width((0, 0), (5, 10), 4)
height = get_height((0, 0), (5, 10), 4)
splist = spiral((0, 0), (5, 10), width, height)
print(splist)



