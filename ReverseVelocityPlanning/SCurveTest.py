import numpy as np
import matplotlib.pyplot as plt

def s_curve(t, acceleration_time, constant_velocity_time):
    """
    Generate an S-curve velocity profile.

    Parameters:
    - t: Time array
    - acceleration_time: Time taken for acceleration phase
    - constant_velocity_time: Time taken for constant velocity phase

    Returns:
    - Velocity array
    """
    a = 1 / acceleration_time
    b = 1 / constant_velocity_time
    v_max = 1.0  # Maximum velocity

    # Calculate acceleration and deceleration segments
    acceleration_segment = 0.5 * a * t**2
    deceleration_segment = v_max * (t - acceleration_time) + 0.5 * b * (t - acceleration_time)**2

    # Combine segments to form the S-curve profile
    velocity = np.piecewise(t, [t < acceleration_time, (t >= acceleration_time) & (t < acceleration_time + constant_velocity_time), t >= acceleration_time + constant_velocity_time],
                            [lambda t: a * t, v_max, lambda t: v_max - b * (t - acceleration_time)])

    return velocity

# Time parameters
total_time = 10.0
num_points = 1000
time = np.linspace(0, total_time, num_points)

# S-curve parameters
acceleration_time = 2.0
constant_velocity_time = 4.0

# Generate S-curve velocity profile
velocity_profile = s_curve(time, acceleration_time, constant_velocity_time)

# Plot the velocity profile
plt.plot(time, velocity_profile, label='Velocity Profile')
plt.title('S-Curve Velocity Profile')
plt.xlabel('Time (s)')
plt.ylabel('Velocity')
plt.legend()
plt.grid(True)
plt.show()
