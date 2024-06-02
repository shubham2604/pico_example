import numpy as np

def is_valid_data(row):
    try:
        # Attempt to convert the row values to float
        float(row[0])
        float(row[1])
        float(row[2])
        return True
    except ValueError:
        return False

# Load raw magnetometer data from data.txt
with open("data.txt", "r") as file:
    lines = file.readlines()

cleaned_lines = [line.strip().split() for line in lines if is_valid_data(line.strip().split())]

# Convert cleaned data lines to a numpy array
cleaned_data = np.array(cleaned_lines, dtype=float)

# Save the cleaned data to cleaned_data.txt
np.savetxt("cleaned_data.txt", cleaned_data, fmt="%.6f", delimiter=" ")

print("Cleaned data saved to cleaned_data.txt")
