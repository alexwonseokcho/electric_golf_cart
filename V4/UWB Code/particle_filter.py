import numpy as np
import matplotlib.pyplot as plt
import random
import time
# Constants
sensor1_pos = np.array([0, 0])  # Position of sensor 1 (origin)
sensor2_pos = np.array([10, 0])  # Position of sensor 2 (10 meters away)
num_particles = 1000  # Number of particles in the filter
noise_std = 0.1  # Measurement noise standard deviation

# Particle class to hold position and weight
class Particle:
    def __init__(self, x, y, weight=1.0):
        self.x = x
        self.y = y
        self.weight = weight

# Function to calculate the distance from a particle to each sensor
def calculate_distances(particle, sensor1_pos, sensor2_pos):
    dist1 = np.sqrt((particle.x - sensor1_pos[0])**2 + (particle.y - sensor1_pos[1])**2)
    dist2 = np.sqrt((particle.x - sensor2_pos[0])**2 + (particle.y - sensor2_pos[1])**2)
    return dist1, dist2

# Function to calculate the likelihood of a particle given the actual measurements
def update_particle_weight(particle, measured_dist1, measured_dist2):
    predicted_dist1, predicted_dist2 = calculate_distances(particle, sensor1_pos, sensor2_pos)
    
    # Add measurement noise to predicted distances
    noise1 = np.random.normal(0, noise_std)
    noise2 = np.random.normal(0, noise_std)
    
    predicted_dist1 += noise1
    predicted_dist2 += noise2
    
    # Calculate the weight as the inverse of the squared error
    error1 = (predicted_dist1 - measured_dist1) ** 2
    error2 = (predicted_dist2 - measured_dist2) ** 2
    
    # Combine the errors for both sensors
    weight = np.exp(-(error1 + error2) / (2 * noise_std**2))
    particle.weight = weight
    return weight

# Function to resample particles based on their weights
def resample(particles):
    weights = np.array([particle.weight for particle in particles])
    total_weight = np.sum(weights)
    normalized_weights = weights / total_weight  # Normalize weights

    # Resample particles based on normalized weights
    indices = np.random.choice(range(len(particles)), size=len(particles), p=normalized_weights)
    resampled_particles = [particles[i] for i in indices]
    return resampled_particles

# Initialize particles randomly within a given area
def initialize_particles(num_particles, area_size=(10, 10)):
    particles = []
    for _ in range(num_particles):
        x = random.uniform(0, area_size[0])
        y = random.uniform(0, area_size[1])
        particles.append(Particle(x, y))
    return particles

# Main loop to run the particle filter
def particle_filter(measured_dist1, measured_dist2, particles):
    # Prediction step (Add noise or movement model here if needed)
    # For now, we just leave particles as is. This can be expanded for dynamic systems.
    
    # Update weights based on the measurements
    for particle in particles:
        update_particle_weight(particle, measured_dist1, measured_dist2)

    # Resample particles based on weights
    particles = resample(particles)

    # Compute the estimated position as the weighted average of all particles
    estimated_position = np.mean([(p.x, p.y) for p in particles], axis=0)
    return estimated_position, particles

# Visualize the particles and estimated position
def plot_particles(particles, estimated_position):
    plt.figure(figsize=(8, 8))
    x_vals = [p.x for p in particles]
    y_vals = [p.y for p in particles]
    
    plt.scatter(x_vals, y_vals, color='blue', alpha=0.2, label='Particles')
    plt.scatter(estimated_position[0], estimated_position[1], color='red', marker='X', label='Estimated Position')
    plt.scatter(tag_position[0], tag_position[1], color='green', marker='o', label='True Position')
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.show()

# Initialize particles
particles = initialize_particles(num_particles)

while True:
    # Simulate some real UWB tag position and calculate the distances
    tag_position = np.array([6, 3])  # Example true position of the tag
    measured_dist1 = np.sqrt((tag_position[0] - sensor1_pos[0])**2 + (tag_position[1] - sensor1_pos[1])**2) + np.random.normal(0, noise_std)
    measured_dist2 = np.sqrt((tag_position[0] - sensor2_pos[0])**2 + (tag_position[1] - sensor2_pos[1])**2) + np.random.normal(0, noise_std)


    # Run the particle filter
    estimated_position, particles = particle_filter(measured_dist1, measured_dist2, particles)


    # Plot particles and estimated position
    plot_particles(particles, estimated_position)
    time.sleep(0.1)