import numpy as np

def differential_drive_regulation_controller(
    current_position,
    current_orientation,
    desired_position,
    desired_orientation,
    wheel_radius,
    wheel_base_width,
    kp_pos, 
    kp_ori,
    max_linear_velocity=100,
    max_angular_velocity=100
):
    """
    Computes the wheel velocities to regulate the robot to the desired position and orientation.

    Parameters:
    - current_position: tuple (x, y)
        The current position of the robot.
    - current_orientation: float
        The current orientation (theta) of the robot in radians.
    - desired_position: tuple (x_d, y_d)
        The desired position for the robot.
    - desired_orientation: float
        The desired orientation (theta_d) of the robot in radians.
    - wheel_radius: float
        The radius of the wheels.
    - wheel_base_width: float
        The distance between the two wheels.
    - max_linear_velocity: float
        The maximum linear speed of the robot.
    - max_angular_velocity: float
        The maximum angular speed of the robot.

    Returns:
    - left_wheel_velocity: float
        The angular velocity for the left wheel (rad/s).
    - right_wheel_velocity: float
        The angular velocity for the right wheel (rad/s).
    """
    # Compute position errors
    error_x = desired_position[0] - current_position[0]
    error_y = desired_position[1] - current_position[1]
    distance_error = np.hypot(error_x, error_y)

    # Compute the angle to the desired position
    angle_to_goal = np.arctan2(error_y, error_x)

    # Compute orientation errors
    orientation_error = desired_orientation - current_orientation
    orientation_error = np.arctan2(np.sin(orientation_error), np.cos(orientation_error))  # Normalize to [-pi, pi]

    # Compute heading error
    heading_error = angle_to_goal - current_orientation
    heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

    # Controller gains (tunable parameters)
    Kp_linear = kp_pos    # Proportional gain for linear velocity
    Kp_angular = kp_ori  # Proportional gain for angular velocity

    # Compute the desired linear and angular velocities
    # Adjust linear velocity based on heading error to slow down when not facing the goal
    adjusted_linear_velocity = Kp_linear * distance_error * np.cos(heading_error)
    adjusted_angular_velocity = Kp_angular * heading_error + Kp_angular * 0.5 * orientation_error

    # Limit velocities to maximum values
    desired_linear_velocity = np.clip(adjusted_linear_velocity, -max_linear_velocity, max_linear_velocity)
    desired_angular_velocity = np.clip(adjusted_angular_velocity, -max_angular_velocity, max_angular_velocity)

    # Compute individual wheel linear velocities
    v_left = desired_linear_velocity - (desired_angular_velocity * wheel_base_width / 2)
    v_right = desired_linear_velocity + (desired_angular_velocity * wheel_base_width / 2)

    # Convert linear velocities to angular velocities (rad/s)
    left_wheel_velocity = v_left / wheel_radius
    right_wheel_velocity = v_right / wheel_radius

    return left_wheel_velocity, right_wheel_velocity
