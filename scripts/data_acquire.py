import serial

# Open the serial port
ser = serial.Serial('/dev/ttyACM0')

# Create a file to save the data
file_path = 'data.txt'
file = open(file_path, 'w')

try:
    while True:
        # Read a line of data from the serial port
        line = ser.readline().decode().strip()

        # Split the line into individual values
        values = line.split(' ')

        if len(values) == 3:
            try:
                # Convert the values to floats
                floats = [float(value) for value in values]

                # Write the floats to the file
                file.write(' '.join(str(f) for f in floats) + '\n')

                # Flush the file buffer
                file.flush()

                print("Data saved:", floats)

            except ValueError:
                print("Invalid data format:", line)
        else:
            print("Invalid data format:", line)

except KeyboardInterrupt:
    print("Program terminated.")

finally:
    # Close the file and serial port
    file.close()
    ser.close()

