import math

def generate_trajectory_turn(direction: int) -> list:
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
            
            point = [x, y, theta, t]
            
            trajectory_points.append(point)
    
    elif direction == 1: # left turn
        duration = 0.3 / speed  # 20 cm straight
        num_points = int(duration * num_points_per_meter)
        for i in range(num_points + 1):
            t = i * (duration / num_points)
            x = speed * t - 0.15
            y = 0
            theta = 0  # Pointing along the positive x-axis
            
            point = [x, y, theta, t]
            
            trajectory_points.append(point)        

        end_x = x
        end_y = y

        turn_radius = 0.3  # 30 cm radius
        arc_length = math.pi / 2 * turn_radius
        turn_duration = arc_length / speed
        num_turn_points = int(turn_duration * num_points_per_meter)
        
        for i in range(num_turn_points + 1):
            t = i * (turn_duration / num_turn_points)
            theta = (math.pi / 2) * (t / turn_duration)  # Turning left
            x = turn_radius * math.sin(theta) + end_x
            y = turn_radius * (1 - math.cos(theta)) + end_y
            
            point = [x, y, theta, t]
            
            trajectory_points.append(point)
        end_y = y
        duration = 0.15 / speed  # 15 cm straight
        for i in range(num_points + 1):
            t = i * (duration / num_points)
            y = speed * t + end_y
            
            point = [x, y, theta, t]
            
            trajectory_points.append(point)   
    
    elif direction == -1: # right turn
        turn_radius = 0.15  # 15 cm radius
        arc_length = math.pi / 2 * turn_radius
        turn_duration = arc_length / speed
        num_turn_points = int(turn_duration * num_points_per_meter)
        duration = 0.15 / speed  # 15 cm straight
        num_points = int(duration * num_points_per_meter)
        for i in range(num_points + 1):
            t = i * (duration / num_points)
            x = speed * t - 0.15
            y = 0
            theta = 0  # Pointing along the positive x-axis
            
            point = [x, y, theta, t]
            
            trajectory_points.append(point)        

        end_x = x
        end_y = y        
        
        for i in range(num_turn_points + 1):
            t = i * (turn_duration / num_turn_points)
            theta = -(math.pi / 2) * (t / turn_duration)  # Turning right
            y = -turn_radius * (1 - math.cos(theta))
            x = -turn_radius * math.sin(theta)

            point = [x, y, theta, t]
            
            trajectory_points.append(point)

    return trajectory_points


