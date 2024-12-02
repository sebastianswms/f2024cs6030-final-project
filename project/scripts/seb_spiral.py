import csv
import math

# Input Parameters
FIRST_CORNER = (305, 241)
SECOND_CORNER = (264, 179)
SNOWPLOW_WIDTH = 4
WAYPOINT_SPACING = 0.5
TURNING_RADIUS = 6

def frange(start, stop, step):
    """Generates a range of floating point numbers with a given step."""
    numbers = []
    if step > 0:
        while start <= stop + 1e-6:
            numbers.append(round(start, 10))
            start += step
    elif step < 0:
        while start >= stop - 1e-6:
            numbers.append(round(start, 10))
            start += step
    else:
        raise ValueError("Step must not be zero.")
    return numbers

def generate_quarter_circle(center_x, center_y, start_angle, end_angle, radius):
    """
    Generates waypoints along a quarter-circle arc from start_angle to end_angle.
    
    Args:
        center_x (float): X-coordinate of the circle's center.
        center_y (float): Y-coordinate of the circle's center.
        start_angle (float): Starting angle in radians.
        end_angle (float): Ending angle in radians.
        radius (float): Radius of the quarter-circle.
    
    Returns:
        list of tuple: List of (x, y) waypoints along the quarter-circle.
    """
    waypoints = []
    step_angle = WAYPOINT_SPACING / radius  # Angle increment based on desired spacing

    if start_angle < end_angle:
        angle = start_angle
        while angle <= end_angle:
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            waypoints.append((round(x, 3), round(y, 3)))
            angle += step_angle
    else:
        angle = start_angle
        while angle >= end_angle:
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            waypoints.append((round(x, 3), round(y, 3)))
            angle -= step_angle

    # Ensure the end_angle point is included
    x = center_x + radius * math.cos(end_angle)
    y = center_y + radius * math.sin(end_angle)
    waypoints.append((round(x, 3), round(y, 3)))

    return waypoints

def generate_straight_waypoints(start_x, start_y, end_x, end_y, spacing):
    """
    Generates waypoints along a straight line from (start_x, start_y) to (end_x, end_y).
    
    Args:
        start_x (float): Starting X-coordinate.
        start_y (float): Starting Y-coordinate.
        end_x (float): Ending X-coordinate.
        end_y (float): Ending Y-coordinate.
        spacing (float): Distance between consecutive waypoints.
    
    Returns:
        list of tuple: List of (x, y) waypoints along the straight line.
    """
    waypoints = []
    if start_x != end_x:
        step = spacing if end_x > start_x else -spacing
        x_coords = frange(start_x, end_x, step)
        waypoints = [(round(x, 3), round(start_y, 3)) for x in x_coords]
    elif start_y != end_y:
        step = spacing if end_y > start_y else -spacing
        y_coords = frange(start_y, end_y, step)
        waypoints = [(round(start_x, 3), round(y, 3)) for y in y_coords]
    return waypoints

def push_snow_out(current_x, current_y, push_end_x, push_end_y, spacing):
    """
    Generates waypoints to push snow out to the parking lot's edge and then reverse back.
    
    Args:
        current_x (float): Current X-coordinate.
        current_y (float): Current Y-coordinate.
        push_end_x (float): X-coordinate of the parking lot's edge.
        push_end_y (float): Y-coordinate of the parking lot's edge.
        spacing (float): Distance between consecutive waypoints.
    
    Returns:
        list of tuple: List of waypoints for pushing snow out and reversing back.
    """
    # Push out to the edge
    push_out_waypoints = generate_straight_waypoints(current_x, current_y, push_end_x, push_end_y, spacing)
    
    # Reverse back to the original point (excluding the last point to avoid duplication)
    push_in_waypoints = push_out_waypoints[:-1][::-1]
    
    return push_out_waypoints + [(-1,-1)] + push_in_waypoints + [(-1,-1)]

def generate_waypoints(first_corner, second_corner):
    """
    Generates a list of waypoints for the snowplow to follow a spiral path with smooth turns.
    
    Args:
        first_corner (tuple): (x, y) coordinates of the first corner.
        second_corner (tuple): (x, y) coordinates of the second corner.
    
    Returns:
        list of tuple: List of (x, y) waypoints defining the snowplow's path.
    """
    x1, y1 = first_corner
    x2, y2 = second_corner

    # Define parking lot boundaries
    x_min = min(x1, x2)
    x_max = max(x1, x2)
    y_min = min(y1, y2)
    y_max = max(y1, y2)

    # Insets to account for the snowplow's width
    x_inset = SNOWPLOW_WIDTH / 2
    y_inset = SNOWPLOW_WIDTH / 2

    # Define initial boundaries for the spiral
    left = x_min + x_inset
    right = x_max - x_inset
    bottom = y_min + y_inset
    top = y_max - y_inset

    # Starting position: bottom-left corner inset by snowplow width
    x, y = left, bottom
    waypoints = []

    # Define direction indices: 0 = right, 1 = up, 2 = left, 3 = down
    current_dir = 0

    bottom += SNOWPLOW_WIDTH

    # Continue spiraling until boundaries overlap considering turning radius
    while (left + TURNING_RADIUS <= right - TURNING_RADIUS) and (bottom + TURNING_RADIUS <= top - TURNING_RADIUS):
        for _ in range(4):  # Iterate through all four directions
            if current_dir == 0:  # Moving Right
                # Define straight path up to the turn start point
                straight_end_x = right - TURNING_RADIUS
                straight_end_y = y
                straight_waypoints = generate_straight_waypoints(x, y, straight_end_x, straight_end_y, WAYPOINT_SPACING)
                waypoints += straight_waypoints
                x = straight_end_x
                y = straight_end_y

                # Push snow out to the right edge
                push_end_x = x_max
                push_end_y = y
                waypoints += push_snow_out(x, y, push_end_x, push_end_y, WAYPOINT_SPACING)

                # Generate quarter-circle turn to Up direction
                center_x = right - TURNING_RADIUS
                center_y = y + TURNING_RADIUS
                start_angle = -math.pi / 2  # 270 degrees
                end_angle = 0               # 0 degrees
                turn_waypoints = generate_quarter_circle(center_x, center_y, start_angle, end_angle, TURNING_RADIUS)
                waypoints += turn_waypoints

                # Update position to the end of the turn
                x = center_x + TURNING_RADIUS * math.cos(end_angle)
                y = center_y + TURNING_RADIUS * math.sin(end_angle)

                # Update direction to Up
                current_dir = 1

            elif current_dir == 1:  # Moving Up
                # Define straight path up to the turn start point
                straight_end_x = x
                straight_end_y = top - TURNING_RADIUS
                straight_waypoints = generate_straight_waypoints(x, y, straight_end_x, straight_end_y, WAYPOINT_SPACING)
                waypoints += straight_waypoints
                x = straight_end_x
                y = straight_end_y

                # Push snow out to the top edge
                push_end_x = x
                push_end_y = y_max
                waypoints += push_snow_out(x, y, push_end_x, push_end_y, WAYPOINT_SPACING)

                # Generate quarter-circle turn to Left direction
                center_x = x - TURNING_RADIUS
                center_y = top - TURNING_RADIUS
                start_angle = 0               # 0 degrees
                end_angle = math.pi / 2       # 90 degrees
                turn_waypoints = generate_quarter_circle(center_x, center_y, start_angle, end_angle, TURNING_RADIUS)
                waypoints += turn_waypoints

                # Update position to the end of the turn
                x = center_x + TURNING_RADIUS * math.cos(end_angle)
                y = center_y + TURNING_RADIUS * math.sin(end_angle)

                # Update direction to Left
                current_dir = 2

            elif current_dir == 2:  # Moving Left
                # Define straight path up to the turn start point
                straight_end_x = left + TURNING_RADIUS
                straight_end_y = y
                straight_waypoints = generate_straight_waypoints(x, y, straight_end_x, straight_end_y, WAYPOINT_SPACING)
                waypoints += straight_waypoints
                x = straight_end_x
                y = straight_end_y

                # Push snow out to the left edge
                push_end_x = x_min
                push_end_y = y
                waypoints += push_snow_out(x, y, push_end_x, push_end_y, WAYPOINT_SPACING)

                # Generate quarter-circle turn to Down direction
                center_x = left + TURNING_RADIUS
                center_y = y - TURNING_RADIUS
                start_angle = math.pi / 2       # 90 degrees
                end_angle = math.pi             # 180 degrees
                turn_waypoints = generate_quarter_circle(center_x, center_y, start_angle, end_angle, TURNING_RADIUS)
                waypoints += turn_waypoints

                # Update position to the end of the turn
                x = center_x + TURNING_RADIUS * math.cos(end_angle)
                y = center_y + TURNING_RADIUS * math.sin(end_angle)

                # Update direction to Down
                current_dir = 3

            elif current_dir == 3:  # Moving Down
                # Define straight path up to the turn start point
                straight_end_x = x
                straight_end_y = bottom + TURNING_RADIUS
                straight_waypoints = generate_straight_waypoints(x, y, straight_end_x, straight_end_y, WAYPOINT_SPACING)
                waypoints += straight_waypoints
                x = straight_end_x
                y = straight_end_y

                # Push snow out to the bottom edge
                push_end_x = x
                push_end_y = y_min
                waypoints += push_snow_out(x, y, push_end_x, push_end_y, WAYPOINT_SPACING)

                # Generate quarter-circle turn to Right direction
                center_x = x + TURNING_RADIUS
                center_y = bottom + TURNING_RADIUS
                start_angle = math.pi           # 180 degrees
                end_angle = 3 * math.pi / 2     # 270 degrees
                turn_waypoints = generate_quarter_circle(center_x, center_y, start_angle, end_angle, TURNING_RADIUS)
                waypoints += turn_waypoints

                # Update position to the end of the turn
                x = center_x + TURNING_RADIUS * math.cos(end_angle)
                y = center_y + TURNING_RADIUS * math.sin(end_angle)

                # Update direction to Right
                current_dir = 0

        # After completing a full loop, inset the boundaries for the next spiral layer
        left += SNOWPLOW_WIDTH
        right -= SNOWPLOW_WIDTH
        bottom += SNOWPLOW_WIDTH
        top -= SNOWPLOW_WIDTH

    return waypoints

if __name__ == "__main__":
    waypoints = generate_waypoints(
        first_corner=FIRST_CORNER,
        second_corner=SECOND_CORNER
    )

    # Output to CSV
    with open('waypoints.csv', 'w', newline='') as csvfile:
        fieldnames = ['x', 'y']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        previous_points = []  # To track the last two waypoints

        toggle = True

        for x, y in waypoints:
            if (x, y) == (-1, -1):
                writer.writerow({'x': '===', 'y': '==='})
                writer.writerow({'x': 'x', 'y': 'y'})
                continue
            writer.writerow({'x': x, 'y': y})
            previous_points.append((x, y))
