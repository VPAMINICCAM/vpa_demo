import sys
import os
from unittest.mock import MagicMock
import matplotlib.pyplot as plt
import numpy as np

# Mock rospy and its components
sys.modules['rospy'] = MagicMock()
sys.modules['nav_msgs.msg'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['vpa_demo.msg'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()

# Add the planning and middleware directories to the system path
sys.path.append(os.path.join(os.path.dirname(__file__), '../planning'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../middleware'))

from trajectory_generation import generate_trajectory_turn
from pure_pursuit import PurePursuitController

def run_test(direction, title, ax_traj, ax_speed, ax_yaw):
    # Generate the trajectory with debug flag set to True
    trajectory_points = generate_trajectory_turn(direction)
    
    # Convert trajectory points to a list of objects with attributes
    class TrajectoryPoint:
        def __init__(self, x, y, theta, time_from_start):
            self.x = x
            self.y = y
            self.theta = theta
            self.time_from_start = time_from_start
    
    trajectory_points = [TrajectoryPoint(*point) for point in trajectory_points]
    
    # Initialize the Pure Pursuit Controller
    lookahead_time = 0.5  # seconds
    max_speed = 1.0  # m/s
    controller = PurePursuitController(trajectory_points, lookahead_time, max_speed)
    
    # Initial state of the robot
    # The state includes the initial position (x, y) and orientation (yaw) of the robot
    class State:
        def __init__(self, x, y, yaw):
            self.x = x
            self.y = y
            self.yaw = yaw
    
    # Set the initial state of the robot
    # Here, the robot starts at position (-0.08, 0.08) with an orientation of 0 radians
    state = State(-0.08, 0.08, 0.0)
    current_time = 0.0
    dt = 1.0 / 20.0  # 20Hz update rate
    
    # Lists to store the trajectory and control commands for visualization
    actual_x = [state.x]
    actual_y = [state.y]
    speeds = []
    yaw_rates = []
    times = []
    
    # Run the simulation for a fixed duration
    simulation_duration = 10.0  # seconds
    while current_time < simulation_duration:
        # Compute the control commands based on the current state and time
        desired_speed, yaw_rate = controller.compute_control(state, current_time)
        
        # Update the state based on the control commands
        state.x += desired_speed * dt * np.cos(state.yaw)
        state.y += desired_speed * dt * np.sin(state.yaw)
        state.yaw += yaw_rate * dt
        
        # Store the actual trajectory and control commands
        actual_x.append(state.x)
        actual_y.append(state.y)
        speeds.append(desired_speed)
        yaw_rates.append(yaw_rate)
        times.append(current_time)
        
        # Update the current time
        current_time += dt
    
    # Plot the generated trajectory and the actual trajectory
    traj_x = [point.x for point in trajectory_points]
    traj_y = [point.y for point in trajectory_points]
    
    ax_traj.plot(traj_x, traj_y, 'ro-', label='Generated Trajectory')
    ax_traj.plot(actual_x, actual_y, 'bo-', label='Actual Trajectory')
    ax_traj.set_xlabel('X')
    ax_traj.set_ylabel('Y')
    ax_traj.set_title(f'{title} Trajectory')
    ax_traj.legend()
    ax_traj.axis('equal')
    ax_traj.set_xlim([-0.6, 0.6])
    ax_traj.set_ylim([-0.6, 0.6])
    ax_traj.grid(True)
    
    # Plot the linear speed and yaw rate curves
    ax_speed.plot(times, speeds, 'b-', label='Linear Speed')
    ax_speed.set_xlabel('Time [s]')
    ax_speed.set_ylabel('Speed [m/s]')
    ax_speed.set_title(f'{title} Linear Speed')
    ax_speed.legend()
    ax_speed.grid(True)
    
    ax_yaw.plot(times, yaw_rates, 'r-', label='Yaw Rate')
    ax_yaw.set_xlabel('Time [s]')
    ax_yaw.set_ylabel('Yaw Rate [rad/s]')
    ax_yaw.set_title(f'{title} Yaw Rate')
    ax_yaw.legend()
    ax_yaw.grid(True)

def test_right_turn(ax_traj, ax_speed, ax_yaw):
    run_test(-1, 'Right Turn', ax_traj, ax_speed, ax_yaw)

def test_through(ax_traj, ax_speed, ax_yaw):
    run_test(0, 'Straight Through', ax_traj, ax_speed, ax_yaw)

def test_left_turn(ax_traj, ax_speed, ax_yaw):
    run_test(1, 'Left Turn', ax_traj, ax_speed, ax_yaw)

if __name__ == '__main__':
    fig, axs = plt.subplots(2, 3, figsize=(15, 10))
    
    test_right_turn(axs[0, 0], axs[1, 0], axs[1, 1])
    test_through(axs[0, 1], axs[1, 1], axs[1, 2])
    test_left_turn(axs[0, 2], axs[1, 2], axs[1, 2])
    
    plt.tight_layout()
    plt.show()




