
import serial
import math

# Set up the serial communication with the sensor
sensor = serial.Serial('COM5', 115200)
sensor.open()
sensor.reset_output_buffer()
sensor.reset_input_buffer()

# Open the output file to save the scan data
file = open("point_array.xyz", "w")

# Initialize scan parameters
step_count = 0
x_position = 0  # Starting x-displacement in mm
x_step_size = 500  # Increment of x-displacement (mm)
num_scans = int(input("Enter the number of scans to perform: "))
scan_counter = 0

# Loop through each scan
while scan_counter < num_scans:
    # Read raw data from the sensor
    raw_data = sensor.readline()
    decoded_data = raw_data.decode("utf-8")
    decoded_data = decoded_data[0:-2] 

    # Check if the data is a valid number (distance reading)
    if decoded_data.isdigit():
        # Calculate the angle of the current sensor reading
        angle = (step_count / 512) * 2 * math.pi  # Full rotation of 360° over 512 steps
        radius = int(decoded_data)
        
        # Calculate Cartesian coordinates
        y_coord = radius * math.cos(angle)  # Y-coordinate based on polar to Cartesian conversion
        z_coord = radius * math.sin(angle)  # Z-coordinate based on polar to Cartesian conversion
        
        # Print the Y and Z coordinates for debugging
        print(y_coord)
        print(z_coord)
        
        # Write the X, Y, and Z coordinates to the output file
        file.write('{} {} {}\n'.format(x,y,z))
        
        # Increment the step count for the next reading
        step_count += 32

    # If one full rotation is completed, reset the step counter and increment X position
    if step_count == 512:
        step_count = 0
        x_position += x_step_size
        scan_counter += 1

    # Optionally print the raw data for debugging
    print(decoded_data)

# Close the file to save all data
file.close()
