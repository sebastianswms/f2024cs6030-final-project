"""
1 get waypoints
2 get max_loops
3 use max_loops in a loop along with a counter to loop through every loop
4 increiment the counter by 1 to increase the loop number, add half the plow width every loop 
5 add extra logic at the end of the cycle 
"""
import csv # used for writing the waypoints file


FIRST_CORNER = (0, 0)
SECOND_CORNER = (10, 20)
SNOWPLOW_WIDTH = 4

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

"""finds center of snowplow"""
def get_spcenter(snowplow_width):
    spcenter = snowplow_width / 2
    return round(spcenter)

"""function to build the spiral"""
def spiral(TlCorner, rectwidth, rectheight, spcenter):
    spirallist = []  # List to hold every point in the spiral
    current_direction = 0  # Start moving right

    left_bound = 0 + spcenter  # Used to see how far left the spiral can go
    right_bound = rectwidth - spcenter  # Used to see how far right the spiral can go
    top_bound = 0 + spcenter  # Used to see how far up the spiral can go
    bottom_bound = rectheight - spcenter  # Used to see how far down the spiral can go
    
    # All static values are used for moving to the edge of the spiral 
    Static_lbound = 0
    Static_rbound = rectwidth
    Static_tbound = 0
    Static_bbound = rectheight

    for i in range(0, left_bound): # adds the extra points to get into the spiral
        print("startingpath")
        spirallist.append((left_bound, i))

    while left_bound <= right_bound and top_bound <= bottom_bound:
        backup = []
        if current_direction == 0:  # Moving right
            for i in range(left_bound, right_bound + 1):
                if i == right_bound:
                    turn = (TlCorner[0] + i, TlCorner[1] + top_bound)
                spirallist.append((TlCorner[0] + i, TlCorner[1] + top_bound))
            
            # Go to the right edge and back
            for i in range(right_bound + 1, Static_rbound + 1):
                spirallist.append((TlCorner[0] + i, TlCorner[1] + top_bound))
                backup.append((TlCorner[0] + i, TlCorner[1] + top_bound))
            
            for i in range(len(backup) - 1, -1, -1):
                spirallist.append(backup[i])
            
            spirallist.append(turn)
            top_bound += spcenter

        elif current_direction == 1:  # Moving down
            for i in range(top_bound, bottom_bound + 1):
                if i == bottom_bound:
                    turn = TlCorner[0] + right_bound, TlCorner[1] + i
                spirallist.append((TlCorner[0] + right_bound, TlCorner[1] + i))
            
            # Go to the bottom edge and back
            for i in range(bottom_bound + 1, (Static_bbound) + 1):
                spirallist.append((TlCorner[0] + right_bound, TlCorner[1] + i))
                backup.append((TlCorner[0] + right_bound, TlCorner[1] + i))
            
            for i in range(len(backup) - 1, -1, -1):
                spirallist.append(backup[i])
            
            spirallist.append(turn)
            right_bound -= spcenter

        elif current_direction == 2:  # Moving left
            for i in range(right_bound, left_bound - 1, -1):
                if i == left_bound:
                    turn = TlCorner[0] + i, TlCorner[1] + bottom_bound
                spirallist.append((TlCorner[0] + i, TlCorner[1] + bottom_bound))
            
            # Go to the left edge and back
            for i in range(left_bound - 1, Static_lbound - 1, -1):
                spirallist.append((TlCorner[0] + i, TlCorner[1] + bottom_bound))
                backup.append((TlCorner[0] + i, TlCorner[1] + bottom_bound))
            
            for i in range(len(backup) - 1, -1, -1):
                spirallist.append(backup[i])
            
            spirallist.append(turn)
            bottom_bound -= spcenter

        elif current_direction == 3:  # Moving up
            for i in range(bottom_bound, top_bound - 1, -1):
                if i == top_bound:
                    turn = TlCorner[0] + left_bound, TlCorner[1] + i
                spirallist.append((TlCorner[0] + left_bound, TlCorner[1] + i))
            
            # Go to the top edge and back
            for i in range(top_bound - 1, Static_tbound - 1, -1):
                spirallist.append((TlCorner[0] + left_bound, TlCorner[1] + i))
                backup.append((TlCorner[0] + left_bound, TlCorner[1] + i))
            
            for i in range(len(backup) - 1, -1, -1):
                spirallist.append(backup[i])
            
            spirallist.append(turn)
            left_bound += spcenter

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
    second_corner = SECOND_CORNER
    snowplow_width = SNOWPLOW_WIDTH
    waypoints = generate_waypoints(first_corner, second_corner)
    width = get_width(first_corner, second_corner)
    height = get_height(first_corner, second_corner)
    spcenter = get_spcenter(snowplow_width)
    path = spiral(first_corner, width, height, spcenter)
    waypoints_writer(path)
    print(path)

