
import numpy as np
import matplotlib.pyplot as plt

from scipy.integrate import cumtrapz
from scipy.interpolate import interp1d


def generate_figure_eight_trajectory(tfinal, dt, translation, theta, A=20, B=10, C=15):
    t = np.arange(0, tfinal, dt)
    x = A * np.sin(t/C)
    y = B * np.sin(t/C) * np.cos(t/C)
    positions = np.vstack((x, y)).T

    # Calculate headings
    headings = np.arctan2(np.gradient(y), np.gradient(x))
    headings = np.unwrap(headings)

    # Calculate velocities
    dx = np.gradient(x, dt)
    dy = np.gradient(y, dt)
    velocity_magnitudes = np.sqrt(dx**2 + dy**2)
    rot_speed = np.gradient(headings,dt)

    ref = np.hstack((positions, headings.reshape(-1, 1), velocity_magnitudes.reshape(-1, 1), rot_speed.reshape(-1,1)))
    ref = transform_trajectory(ref, translation, theta)
    np.save('ref_data.npy',ref)

    return ref


def generate_figure_eight_trajectory_con(tfinal, dt, translation, theta, A=20, B=10, C=15):
    t = np.arange(0, tfinal, dt)
    x = A * np.sin(t/C)
    y = B * np.sin(t/C) * np.cos(t/C)

    # Calculate the arc length for each point
    dx = np.gradient(x)
    dy = np.gradient(y)
    ds = np.sqrt(dx**2 + dy**2)
    s = cumtrapz(ds, initial=0)
    
    # Create a uniform parameter based on the arc length
    s_uniform = np.linspace(0, s[-1], len(s))
    t_uniform = interp1d(s, t)(s_uniform)
    
    # Recompute the positions and velocities using the uniform parameter
    x_uniform = A * np.sin(t_uniform/C)
    y_uniform = B * np.sin(t_uniform/C) * np.cos(t_uniform/C)
    dx_uniform = np.gradient(x_uniform, dt)
    dy_uniform = np.gradient(y_uniform, dt)
    velocity_magnitudes = np.sqrt(dx_uniform**2 + dy_uniform**2)
    
    positions = np.vstack((x_uniform, y_uniform)).T
    headings = np.arctan2(dy_uniform, dx_uniform)
    headings = np.unwrap(headings)
    rot_speed = np.gradient(headings,dt)
    ref = np.hstack((positions, headings.reshape(-1, 1), velocity_magnitudes.reshape(-1, 1), rot_speed.reshape(-1, 1)))
    ref = transform_trajectory(ref, translation, theta)
    np.save('ref_data.npy',ref)

    return ref

def transform_trajectory(trajectory, translation, theta):
    """
    Transform the trajectory by translating and rotating.

    Parameters:
    - trajectory: numpy array of shape (n, 5) where n is the number of points.
    - translation: tuple (tx, ty) specifying the translation vector.
    - theta: rotation angle in radians.

    Returns:
    - Transformed trajectory.
    """
    # Translation
    translated_positions = trajectory[:, :2]

    # Rotation matrix
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    rotation_matrix = np.array([[cos_theta, -sin_theta],
                                [sin_theta, cos_theta]])

    # Rotate positions
    rotated_positions = translated_positions @ rotation_matrix.T  + np.array(translation)

    # Adjust headings by adding rotation angle
    adjusted_headings = trajectory[:, 2] + theta

    # Maintain velocity magnitudes and rotational speeds
    velocity_magnitudes = trajectory[:, 3]
    rot_speed = trajectory[:, 4]

    transformed_trajectory = np.hstack((rotated_positions,
                                        adjusted_headings.reshape(-1, 1),
                                        velocity_magnitudes.reshape(-1, 1),
                                        rot_speed.reshape(-1, 1)))

    return transformed_trajectory



if __name__ == '__main__':
    tfinal = 250
    dt = 0.01
    t = np.arange(0, tfinal, dt)
    positions = generate_figure_eight_trajectory(tfinal, dt, (100,50), 30*np.pi/180)
    positions_con = generate_figure_eight_trajectory_con(tfinal, dt, (100,50), 30*np.pi/180)


    # Save data to files
    np.save('ref_data.npy',positions)
    np.save('ref_data_con.npy',positions_con)
    
    fig, axs = plt.subplots(2, 4, figsize=(12, 10))

    # Plot the figure-eight trajectory
    axs[0, 0].plot(positions[::20,0], positions[::20,1], 'b.')
    axs[0, 0].set_title("Figure-Eight Trajectory")
    axs[0, 0].set_xlabel("X Position")
    axs[0, 0].set_ylabel("Y Position")
    axs[0, 0].grid(True)
    axs[0, 0].axis('equal')

    # Plot the headings
    axs[0, 1].plot(t,positions[:,2], 'r')
    axs[0, 1].set_xlabel("Time")
    axs[0, 1].set_ylabel("Heading (rad)")
    axs[0, 1].grid(True)

    # Plot the headings
    axs[0, 2].plot(t,positions[:,3], 'r')
    axs[0, 2].set_xlabel("Time")
    axs[0, 2].set_ylabel("Velocity (m/s)")
    axs[0, 2].grid(True)

    # Plot the headings
    axs[0, 3].plot(t,positions[:,4], 'r')
    axs[0, 3].set_xlabel("Time")
    axs[0, 3].set_ylabel("Rot speed (rad/s)")
    axs[0, 3].grid(True)

    # Plot the X position over time
    axs[1, 0].plot(positions_con[::20,0], positions_con[::20,1], 'b.')
    axs[1, 0].set_title("Figure-Eight Trajectory")
    axs[1, 0].set_xlabel("X Position")
    axs[1, 0].set_ylabel("Y Position")
    axs[1, 0].grid(True)
    axs[1, 0].axis('equal')


    # Plot the headings
    axs[1, 1].plot(t,positions_con[:,2], 'r')
    axs[1, 1].set_xlabel("Time")
    axs[1, 1].set_ylabel("Heading (rad)")
    axs[1, 1].grid(True)

    # Plot the headings
    axs[1, 2].plot(t,positions_con[:,3], 'r')
    axs[1, 2].set_xlabel("Time")
    axs[1, 2].set_ylabel("Velocity (m/s)")
    axs[1, 2].grid(True)
    axs[1, 2].set_ylim(positions_con[-1,3]-0.2, positions_con[-1,3]+0.2)

    # Plot the headings
    axs[1, 3].plot(t,positions_con[:,4], 'r')
    axs[1, 3].set_xlabel("Time")
    axs[1, 3].set_ylabel("Rot speed (rad/s)")
    axs[1, 3].grid(True)

    plt.tight_layout()
    plt.show()