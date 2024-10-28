"""
1 get waypoints
2 get max_loops
3 use max_loops in a loop along with a counter to loop through every loop
4 increiment the counter by 1 to increase the loop number, add half the plow width every loop 
5 add extra logic at the end of the cycle 
"""
import csv # used for writing the waypoints file


FIRST_CORNER = (0, 0)
SECOND_CORNER = (100, 200)

"""Gemerates waypoints based on a rectangle defined by its corners."""
def generate_waypoints(first_corner, second_corner):
    waypoints = []
    for i in range(second_corner[0] + 1):
        for j in range(second_corner[1] + 1): # plus one so we get 0 to the very back of the rectangle
            waypoints.append((i, j))
    return waypoints


"""finds the exact width of the rectangle"""
def get_width(first_corner, second_corner):
    Width = second_corner[0] - first_corner[0]
    return(round(Width))

"""finds the exact height of the rectangle"""
def get_height(first_corner, second_corner):
    height = second_corner[1] - first_corner[1]
    return(round(height))

"""function to build the spiral"""
def spiral(TlCorner, rectwidth, rectheight):
    spirallist = [] # list to hold every point in the spiral
    current_direction = 0  # start moving right

    left_bound = 0 # used to see how far left the spiral can go 
    right_bound = rectwidth # used to see how far right the spiral can go 
    top_bound = 0 # used to see how far up the spiral can go 
    bottom_bound = rectheight # used to see how far down the spiral can go 
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

def waypoints_writer(path):
    csv_header = ["x", "y"]
    with open("waypoints.csv", "w") as waypoint:
        csv_writer = csv.writer(waypoint)
        csv_writer.writerow(csv_header)
        csv_writer.writerows(path)
    print("waypoints.csv written successfully! ")

if __name__ == "__main__":
    first_corner = FIRST_CORNER 
    second_corner=SECOND_CORNER
    waypoints = generate_waypoints(first_corner, second_corner)
    width = get_width(first_corner, second_corner)
    height = get_height(first_corner, second_corner)
    path = spiral(first_corner, width, height)
    waypoints_writer(path)

