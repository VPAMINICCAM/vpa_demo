import numpy as np

class PurePursuitController:
    """
    Pure Pursuit Controller for Time-Space Tracking.

    This controller selects a target point along a predefined trajectory using a lookahead time offset.
    It computes a desired speed to cover the distance to that target point in the lookahead time,
    and it computes a yaw rate using the pure pursuit method. The yaw rate is saturated to ±3 rad/s.

    Attributes:
        trajectory (list): A list of trajectory points. Each point must have attributes:
                           x, y, theta, and time_from_start.
        lookahead_time (float): The time offset [s] used to select the target point.
        max_speed (float): Maximum allowable speed [m/s].
    """

    def __init__(self, trajectory, lookahead_time, max_speed):
        self.trajectory = trajectory
        self.lookahead_time = lookahead_time
        self.max_speed = max_speed

    def compute_control(self, state, current_time, yaw_rate_bound=3):
        """
        Compute the control commands based on the current state and time.

        Args:
            state: The current vehicle state. Must have attributes: x, y, and yaw.
            current_time (float): The current simulation time in seconds.

        Returns:
            tuple: (desired_speed, yaw_rate)
                   - desired_speed (float): The computed linear speed [m/s].
                   - yaw_rate (float): The computed yaw rate [rad/s], saturated within [-3, 3].
        """
        # Check if the destination is reached (within a threshold)
        final_point = self.trajectory[-1]
        if np.hypot(final_point.x - state.x, final_point.y - state.y) < 0.01:
            return 0.0, 0.0

        # Select the target point based on lookahead time
        target_time = current_time + self.lookahead_time
        target_point = None
        for pt in self.trajectory:
            if pt.time_from_start >= target_time:
                target_point = pt
                break
        if target_point is None:
            target_point = final_point

        # Compute the Euclidean distance to the target point
        d_target = np.hypot(target_point.x - state.x, target_point.y - state.y)
        # Compute the desired speed so that d_target is covered in lookahead_time,
        # but do not exceed the maximum allowed speed.
        desired_speed = np.clip(0.8*d_target / self.lookahead_time, 0, self.max_speed)

        # Transform the target point into the vehicle's coordinate frame.
        dx = target_point.x - state.x
        dy = target_point.y - state.y
        x_local = np.cos(state.yaw) * dx + np.sin(state.yaw) * dy
        y_local = -np.sin(state.yaw) * dx + np.cos(state.yaw) * dy

        # Compute the angle to the target point in the vehicle frame.
        alpha = np.arctan2(y_local, x_local)

        # Compute curvature using the pure pursuit method.
        curvature = 2 * np.sin(alpha) / d_target if d_target != 0 else 0
        yaw_rate = desired_speed * curvature

        # Saturate the yaw rate to ±3 rad/s.
        yaw_rate = np.clip(yaw_rate, -yaw_rate_bound, yaw_rate_bound)

        return desired_speed, yaw_rate
