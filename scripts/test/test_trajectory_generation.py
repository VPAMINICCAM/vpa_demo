import math
import sys
import os
from unittest.mock import MagicMock
import matplotlib.pyplot as plt

# Mock rospy and its components
sys.modules['rospy'] = MagicMock()
sys.modules['nav_msgs.msg'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['vpa_demo.msg'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()

# Add the planning directory to the system path
sys.path.append(os.path.join(os.path.dirname(__file__), '../planning'))

from trajectory_generation import generate_trajectory_turn

def test_generate_trajectory_turn():
    # Define the turn direction (0: straight, 1: left turn, -1: right turn)
    direction = -1  # Right turn
    
    # Generate the trajectory with debug flag set to True
    trajectory_points = generate_trajectory_turn(direction, debug=True)
    
    # Extract the trajectory points for visualization and printing
    traj_x = [point[0] for point in trajectory_points]
    traj_y = [point[1] for point in trajectory_points]
    
    # Print the trajectory points
    print("Trajectory Points:")
    for i, point in enumerate(trajectory_points):
        print(f"Point {i}: x={point[0]}, y={point[1]}, theta={point[2]}, time_from_start={point[3]}")
    
    # Plot the trajectory
    plt.figure()
    plt.plot(traj_x, traj_y, 'ro-', label='Trajectory')
    
    # Draw initial and final headings
    start_x, start_y, start_theta = 0.0, 0.0, 0.0
    end_x, end_y, end_theta = traj_x[-1], traj_y[-1], trajectory_points[-1][2]
    plt.arrow(start_x, start_y, 0.1 * math.cos(start_theta), 0.1 * math.sin(start_theta), head_width=0.05, head_length=0.1, fc='k', ec='k')
    plt.arrow(end_x, end_y, 0.1 * math.cos(end_theta), 0.1 * math.sin(end_theta), head_width=0.05, head_length=0.1, fc='k', ec='k')
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Generated Trajectory')
    plt.show()

if __name__ == '__main__':
    test_generate_trajectory_turn()
