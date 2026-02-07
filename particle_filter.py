import numpy as np
import scipy.stats
import matplotlib.pyplot as plt
import time
import os
from matplotlib.animation import FFMpegWriter
from read_data import read_world, read_sensor_data


def initialize_particles(num_particles, map_limits):
    particles = []
    for i in range(num_particles):
        particle = {
            'x': np.random.uniform(map_limits[0], map_limits[1]),
            'y': np.random.uniform(map_limits[2], map_limits[3]),
            'theta': np.random.uniform(-np.pi, np.pi)
        }
        particles.append(particle)
    return particles


def sample_motion_model(odometry, particles, noise):
    delta_r1 = odometry['r1']
    delta_r2 = odometry['r2']
    delta_t = odometry['t']

    sigma_r1 = noise[0] * abs(delta_r1) + noise[1] * abs(delta_t)
    sigma_t = noise[2] * abs(delta_t) + noise[3] * (abs(delta_r1) + abs(delta_r2))
    sigma_r2 = noise[0] * abs(delta_r2) + noise[1] * abs(delta_t)

    new_particles = []

    for particle in particles:
        delta_r1_tilde = delta_r1 + np.random.normal(0, sigma_r1)
        delta_t_tilde = delta_t + np.random.normal(0, sigma_t)
        delta_r2_tilde = delta_r2 + np.random.normal(0, sigma_r2)

        x_next = particle['x'] + delta_t_tilde * np.cos(particle['theta'] + delta_r1_tilde)
        y_next = particle['y'] + delta_t_tilde * np.sin(particle['theta'] + delta_r1_tilde)
        theta_next = particle['theta'] + delta_r1_tilde + delta_r2_tilde
        theta_next = np.arctan2(np.sin(theta_next), np.cos(theta_next))

        new_particles.append({'x': x_next, 'y': y_next, 'theta': theta_next})

    return new_particles


def eval_sensor_model(sensor_data, particles, landmarks, sigma_r):
    ids = sensor_data['id']
    ranges = sensor_data['range']

    weights = []

    for particle in particles:
        prob = 1.0
        for lm_id, measured_range in zip(ids, ranges):
            dx = particle['x'] - landmarks[lm_id][0]
            dy = particle['y'] - landmarks[lm_id][1]
            expected_range = np.sqrt(dx**2 + dy**2)
            prob *= scipy.stats.norm.pdf(measured_range, expected_range, sigma_r)
        weights.append(prob)

    weights = np.array(weights)
    if np.sum(weights) > 0:
        weights /= np.sum(weights)
    else:
        weights = np.ones(len(particles)) / len(particles)

    return weights


def resample_particles(particles, weights):
    new_particles = []
    num_particles = len(particles)

    start = np.random.uniform(0, 1.0 / num_particles)
    pointers = [start + i / num_particles for i in range(num_particles)]
    cumulative_weights = np.cumsum(weights)

    i = 0
    for pointer in pointers:
        while i < len(cumulative_weights) - 1 and pointer > cumulative_weights[i]:
            i += 1
        new_particles.append(dict(particles[i]))

    return new_particles


def mean_pose(particles):
    xs = [p['x'] for p in particles]
    ys = [p['y'] for p in particles]
    thetas = [p['theta'] for p in particles]
    mean_theta = np.arctan2(np.mean(np.sin(thetas)), np.mean(np.cos(thetas)))
    return [np.mean(xs), np.mean(ys), mean_theta]


def plot_state(particles, landmarks, map_limits):
    xs = [p['x'] for p in particles]
    ys = [p['y'] for p in particles]

    lx = [landmarks[i][0] for i in sorted(landmarks.keys())]
    ly = [landmarks[i][1] for i in sorted(landmarks.keys())]

    estimated_pose = mean_pose(particles)

    plt.clf()
    plt.plot(xs, ys, 'r.', markersize=2, alpha=0.5, label='Particles')
    plt.plot(lx, ly, 'bo', markersize=10, label='Landmarks')

    for i, lm_id in enumerate(sorted(landmarks.keys())):
        plt.annotate(str(lm_id), (lx[i]-0.1, ly[i]-0.6))

    plt.quiver(
        estimated_pose[0], estimated_pose[1],
        np.cos(estimated_pose[2]), np.sin(estimated_pose[2]),
        angles='xy', scale_units='xy', scale=1,
        color='green', width=0.01, label='Estimate'
    )

    plt.axis(map_limits)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Particle Filter Localization')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.pause(0.01)


def main():
    np.random.seed(123)

    fig = plt.figure(figsize=(10, 8))
    plt.ion()
    plt.show()

    os.makedirs("Results", exist_ok=True)
    video_path = os.path.join("Results", "particle_filter_result.mp4")

    writer = FFMpegWriter(fps=10, metadata=dict(artist='Particle Filter'))

    print("Reading landmark positions...")
    landmarks = read_world("data/world.dat")

    print("Reading sensor data...")
    sensor_readings = read_sensor_data("data/sensor_data.dat")

    map_limits = [-1, 12, 0, 11]
    sigma_r = 0.2
    num_particles = 1000
    noise = [0.1, 0.1, 0.05, 0.05]

    particles = initialize_particles(num_particles, map_limits)

    num_timesteps = int(len(sensor_readings) / 2)

    print("Recording simulation video...")

    with writer.saving(fig, video_path, dpi=100):
        for timestep in range(num_timesteps):
            current_odometry = sensor_readings[timestep, 'odometry']
            current_sensor_data = sensor_readings[timestep, 'sensor']

            particles = sample_motion_model(current_odometry, particles, noise)
            weights = eval_sensor_model(current_sensor_data, particles, landmarks, sigma_r)
            particles = resample_particles(particles, weights)

            plot_state(particles, landmarks, map_limits)
            writer.grab_frame()

            time.sleep(0.1)

    print(f"✅ Video saved at: {video_path}")
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
