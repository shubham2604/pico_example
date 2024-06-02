import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import time

# Initialize serial port
ser = serial.Serial()
ser.port = '/dev/ttyACM0'  # Arduino serial port
ser.baudrate = 9600
ser.timeout = 10  # Specify timeout when using readline()
ser.open()
if ser.is_open == True:
    print("\nAll right, serial port now open. Configuration:\n")
    print(ser, "\n")  # Print serial parameters

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []  # Store time values here
x_values = []  # Store x values here
y_values = []  # Store y values here

# This function is called periodically from FuncAnimation
def animate(i, xs, x_values, y_values):

    # Acquire and parse data from serial port
    line = ser.readline().strip().decode('utf-8')
    if line:
        values = list(map(float, line.split()))
        if len(values) == 3:
            timestamp = time.time()  # Get current time
            x_value = values[0]
            y_value = values[1]

            # Add time and values to lists
            xs.append(timestamp)
            x_values.append(x_value)
            y_values.append(y_value)

            # Limit lists to a certain number of items
            max_data_points = 50
            xs = xs[-max_data_points:]
            x_values = x_values[-max_data_points:]
            y_values = y_values[-max_data_points:]

            # Draw x and y plots combined
            ax.clear()
            ax.plot(xs, x_values, label="X vs Time")
            ax.plot(xs, y_values, label="Y vs Time", color='orange')
            ax.set_title('Live X and Y Values vs Time')
            ax.set_xlabel('Time')
            ax.set_ylabel('Value')
            ax.legend()
            ax.set_xlim(min(xs), max(xs))  # Set x-axis limits to show all data

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, x_values, y_values), interval=1)
plt.tight_layout()
plt.show()

# Close the serial connection
ser.close()
