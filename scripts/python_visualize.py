import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Serial port configuration
port = '/dev/ttyACM0'
baud_rate = 9600

# Initialize empty lists for x, y, z measurements
x = []
y = []
z = []

# Create a figure and subplot for plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('Live Magnetometer Data')

# Initialize scatter plot object
scat = ax.scatter([], [], [], label='Raw Data')

# Set initial axes limits based on the range of data
ax.set_xlim(-2000, 2000)
ax.set_ylim(-2000, 2000)
ax.set_zlim(-2000, 2000)

# Initialize variables for soft iron and hard iron calibration
soft_iron_matrix = np.eye(3)
hard_iron_offsets = np.zeros(3)

# Define the update function to be called for each frame of the animation
def update(frame):
    # Read data from serial port
    line = ser.readline().strip().decode('utf-8')
    if line:
        try:
            values = list(map(float, line.split()))
            if len(values) == 3:
                x.append(values[0])
                y.append(values[1])
                z.append(values[2])

                # Update scatter plot data
                scat._offsets3d = (x, y, z)

                # Perform soft iron and hard iron calibration
                calibrated_data = calibrate_data(values)

                # Update calibration ellipse
                update_calibration_ellipse()

        except ValueError:
            pass

# Function to perform soft iron and hard iron calibration
def calibrate_data(data):
    # Apply hard iron calibration
    data = np.array(data) - hard_iron_offsets

    # Apply soft iron calibration
    calibrated_data = np.dot(soft_iron_matrix, data)

    return calibrated_data

# Function to update the calibration ellipse
def update_calibration_ellipse():
    # Compute the covariance matrix
    covariance_matrix = np.cov(np.vstack((x, y, z)))

    # Check if covariance matrix is valid
    if np.all(np.isfinite(covariance_matrix)):
        # Compute the eigenvalues and eigenvectors of the covariance matrix
        eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)

        # Check if eigenvalues are valid and not all zero
        if np.all(np.isfinite(eigenvalues)) and np.any(eigenvalues != 0):
            # Sort the eigenvalues and eigenvectors in descending order
            sorted_indices = np.argsort(eigenvalues)[::-1]
            eigenvalues = eigenvalues[sorted_indices]
            eigenvectors = eigenvectors[:, sorted_indices]

            # Compute the semi-axes lengths as square roots of the eigenvalues
            semi_axes_lengths = np.sqrt(np.maximum(0, eigenvalues))

            # Create the calibration ellipse
            u = np.linspace(0, 2 * np.pi, 100)
            v = np.linspace(0, np.pi, 100)
            x_ellipse = semi_axes_lengths[0] * np.outer(np.cos(u), np.sin(v))
            y_ellipse = semi_axes_lengths[1] * np.outer(np.sin(u), np.sin(v))
            z_ellipse = semi_axes_lengths[2] * np.outer(np.ones_like(u), np.cos(v))

            # Rotate and translate the calibration ellipse based on the eigenvectors and means
            calibration_ellipse = np.zeros_like(x_ellipse)
            for i in range(len(x_ellipse)):
                for j in range(len(x_ellipse[i])):
                    point = np.array([x_ellipse[i, j], y_ellipse[i, j], z_ellipse[i, j]])
                    transformed_point = np.dot(eigenvectors, point)
                    calibration_ellipse[i, j] = transformed_point + np.mean([x, y, z], axis=1)

            # Plot the calibration ellipse
            ax.plot_surface(calibration_ellipse[:, :, 0], calibration_ellipse[:, :, 1], calibration_ellipse[:, :, 2], color='r', alpha=0.3)



# Initialize serial connection
ser = serial.Serial(port, baud_rate, timeout=1)

# Create the animation
ani = FuncAnimation(fig, update, interval=100)

# Display the plot
plt.legend()
plt.show()

# Close the serial connection
ser.close()

