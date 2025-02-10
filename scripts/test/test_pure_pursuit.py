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

# Add the middleware directory to the system path
sys.path.append(os.path.join(os.path.dirname(__file__), '../middleware'))

from pure_pursuit import PurePursuit

class MockPurePursuit(PurePursuit):
    def __init__(self):
        # Initialize parameters directly without ROS
        self.L_min = 0.5
        self.k = 0.5  # Adjusted gain
        self.max_speed = 0.5  # Reduced speed
        self.max_yaw_rate = 0.5  # Reduced yaw rate
        
        self.trajectory = []
        self.current_index = 0  # Track the current index in the trajectory
        self.x_r, self.y_r, self.theta_r, self.v = 0.0, 0.0, 0.0, 0.0
    
    def set_trajectory(self, trajectory):
        self.trajectory = trajectory
        self.current_index = 0  # Reset the current index when a new trajectory is set
    
    def set_odometry(self, x, y, theta, v):
        self.x_r = x
        self.y_r = y
        self.theta_r = theta
        self.v = v
    
    def test_compute_cmd_vel(self):
        pursuit_point = self.find_pursuit_point()
        cmd = self.compute_cmd_vel()
        return cmd, pursuit_point

def calculate_orientation(x1, y1, x2, y2):
    theta = math.atan2(y2 - y1, x2 - x1)
    return theta

def plot_trajectory_and_path(trajectory, path):
    traj_x, traj_y = zip(*[(x, y) for x, y, _, _ in trajectory])
    path_x, path_y = zip(*path)
    
    plt.figure()
    plt.plot(traj_x, traj_y, 'ro-', label='Trajectory')
    plt.plot(path_x, path_y, 'bo-', label='Path')
    
    # Draw initial headings
    for x, y, theta, _ in trajectory:
        plt.arrow(x, y, 0.1 * math.cos(theta), 0.1 * math.sin(theta), head_width=0.05, head_length=0.1, fc='k', ec='k')
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Pure Pursuit Trajectory and Path')
    plt.show()

def test_pure_pursuit():
    pp = MockPurePursuit()
    # Set a sample trajectory including the initial point (0,0,0)
    trajectory = [
        (0.0, 0.0, calculate_orientation(0.0, 0.0, 1.0, 1.0), 0.0),
        (1.0, 1.0, calculate_orientation(1.0, 1.0, 2.0, 2.0), 1.0),
        (2.0, 2.0, calculate_orientation(2.0, 2.0, 3.0, 3.0), 2.0),
        (3.0, 3.0, calculate_orientation(2.0, 2.0, 3.0, 3.0), 3.0)
    ]
    pp.set_trajectory(trajectory)
    
    # Simulate the robot's path
    path = []
    pp.set_odometry(0.0, 0.0, 0.0, 0.5)  # Reduced initial speed
    while True:  # Continue the loop until the robot arrives at the destination
        cmd, pursuit_point = pp.test_compute_cmd_vel()
        if cmd:
            # Update the robot's position based on the command velocity
            pp.x_r += cmd.linear.x * math.cos(pp.theta_r) * 0.1  # Multiply by time step (0.1s)
            pp.y_r += cmd.linear.x * math.sin(pp.theta_r) * 0.1
            pp.theta_r += cmd.angular.z * 0.1
            path.append((pp.x_r, pp.y_r))
            print(f"Linear Velocity: {cmd.linear.x}, Angular Velocity: {cmd.angular.z}")
            print(f"Pursuit Point: {pursuit_point}")
            
            # Check if the robot has arrived at the destination
            if math.sqrt((pp.x_r - trajectory[-1][0])**2 + (pp.y_r - trajectory[-1][1])**2) < 0.1:
                break
        else:
            print("No command velocity computed")
            break
    
    plot_trajectory_and_path(trajectory, path)

if __name__ == "__main__":
    test_pure_pursuit()
