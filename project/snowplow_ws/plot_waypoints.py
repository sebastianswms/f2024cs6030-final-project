import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV files, treating '===', 'x', and 'y' as NaN
na_values = ['===', 'x', 'y']
waypoints = pd.read_csv('waypoints.csv', na_values=na_values)
actual_waypoints = pd.read_csv('actual_waypoints.csv', na_values=na_values)

# Drop rows with NaN values in 'x' or 'y' columns
waypoints_cleaned = waypoints.dropna(subset=['x', 'y'])
actual_waypoints_cleaned = actual_waypoints.dropna(subset=['x', 'y'])

# Convert 'x' and 'y' columns to float
waypoints_cleaned['x'] = waypoints_cleaned['x'].astype(float)
waypoints_cleaned['y'] = waypoints_cleaned['y'].astype(float)
actual_waypoints_cleaned['x'] = actual_waypoints_cleaned['x'].astype(float)
actual_waypoints_cleaned['y'] = actual_waypoints_cleaned['y'].astype(float)

# Create a new plot
plt.figure()

# Plot waypoints
plt.plot(waypoints_cleaned['x'], waypoints_cleaned['y'], label='Waypoints')

# Plot actual waypoints
plt.plot(actual_waypoints_cleaned['x'], actual_waypoints_cleaned['y'], label='Actual Waypoints')

# Set the aspect ratio to 'equal' to lock x and y axis scaling
plt.gca().set_aspect('equal', adjustable='box')

# Add labels and legend
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Waypoints vs Actual Waypoints')
plt.legend()

# Save the plot as 'plot.png'
plt.savefig('plot.png')