import math
import rospy
from geometry_msgs.msg import Point, Quaternion
from vpa_demo.msg import TrajectoryPoint, TimeBasedPath
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray

def generate_trajectory_turn(direction: int, debug=False) -> TimeBasedPath:
    speed = 0.3  # m/s
    num_points_per_meter = 10 / speed  # 10 Hz frequency
    trajectory_points = []
    
    if direction == 0:
        duration = 0.6 / speed  # 60 cm straight
        num_points = int(duration * num_points_per_meter)
        for i in range(num_points + 1):
            t = i * (duration / num_points)
            x = speed * t
            y = 0
            theta = 0  # Pointing along the positive x-axis
            
            if not debug:
                point = TrajectoryPoint()
                point.position = Point(x, y, 0)
                point.orientation = euler_to_quaternion(0, 0, theta)
                point.time_from_start = t
            else:
                point = [x, y, theta, t]
            
            trajectory_points.append(point)
    
    elif direction == 1: # left turn
        turn_radius = 0.3  # 30 cm radius
        arc_length = math.pi / 2 * turn_radius
        turn_duration = arc_length / speed
        num_turn_points = int(turn_duration * num_points_per_meter)
        
        for i in range(num_turn_points + 1):
            t = i * (turn_duration / num_turn_points)
            theta = (math.pi / 2) * (t / turn_duration)  # Turning left
            x = turn_radius * math.sin(theta)
            y = turn_radius * (1 - math.cos(theta))
            
            if not debug:
                point = TrajectoryPoint()
                point.position = Point(x, y, 0)
                point.orientation = euler_to_quaternion(0, 0, theta)
                point.time_from_start = t
            else:
                point = [x, y, theta, t]
            
            trajectory_points.append(point)
    
    elif direction == -1: # right turn
        turn_radius = 0.15  # 15 cm radius
        arc_length = math.pi / 2 * turn_radius
        turn_duration = arc_length / speed
        num_turn_points = int(turn_duration * num_points_per_meter)
        
        for i in range(num_turn_points + 1):
            t = i * (turn_duration / num_turn_points)
            theta = -(math.pi / 2) * (t / turn_duration)  # Turning right
            y = -turn_radius * (1 - math.cos(theta))
            x = -turn_radius * math.sin(theta)
            
            if not debug:
                point = TrajectoryPoint()
                point.position = Point(x, y, 0)
                point.orientation = euler_to_quaternion(0, 0, theta)
                point.time_from_start = t
            else:
                point = [x, y, theta, t]
            
            trajectory_points.append(point)
    
    if debug:
        return trajectory_points
    else:
        trajectory = TimeBasedPath()
        trajectory.header = Header()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.points = trajectory_points
        return trajectory

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return Quaternion(qx, qy, qz, qw)

if __name__ == '__main__':
    rospy.init_node('trajectory_generation_node')
    
    # Define a callback to update the coordinates from a topic
    coordinates = [0.0, 0.0, 0.0, 0.45, 0.60, math.pi / 2]
    
    def coordinates_callback(msg):
        global coordinates
        coordinates = msg.data
    
    # Subscribe to the topic to get the coordinates
    rospy.Subscriber('trajectory_coordinates_topic', Float32MultiArray, coordinates_callback)
    
    # Wait for the first message to be received
    rospy.wait_for_message('trajectory_coordinates_topic', Float32MultiArray)
    
    start_x, start_y, start_theta, end_x, end_y, end_theta = coordinates
    
    # Generate the trajectory with debug flag set to True
    trajectory = generate_trajectory_turn(start_x, start_y, start_theta, end_x, end_y, end_theta, debug=True)
    
    # Publish the trajectory
    traj_pub = rospy.Publisher('trajectory', TimeBasedPath, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        traj_pub.publish(trajectory)
        rate.sleep()
