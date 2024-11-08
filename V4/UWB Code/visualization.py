import serial
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Initialize serial port (replace 'COM3' with your serial port)
ser = serial.Serial('/dev/cu.usbmodem101', 9600, timeout=1)

# Constants
anchor_distance = 0.4  # Distance between the two anchors in meters

# Function to calculate tag position
def calculate_position(d_left, d_right):
    # Using basic geometry to solve for (x, y)
    x = (d_left**2 - d_right**2 + anchor_distance**2) / (2 * anchor_distance)
    y = math.sqrt(abs(d_left**2 - x**2))  # abs to avoid sqrt of negative values
    
    return x, y

# Set up the plot
fig, ax = plt.subplots()
ax.set_xlim(-10, 10)  # Adjust based on expected coordinates
ax.set_ylim(10, 10)
scat, = ax.plot([], [], 'bo')  # Blue dot for the tag position

# while(True):
#     line = ser.readline().decode('utf-8').strip()  # Read from serial
#     try:
#         distances = line.split()
#         print(distances)
#     except:
#         pass


# Update function for live plot
def update(frame):
    line = ser.readline().decode('utf-8').strip()  # Read from serial
    try:
        distances = line.split()
        print(distances)
    except:
        pass
    # if line:
    #     try:
    #         # Extract distances from the serial message
    #         distances = line.split()
    #         d_left = float(distances[0])
    #         d_right = float(distances[1])
            
    #         # Calculate position
    #         x, y = calculate_position(d_left, d_right)
            
    #         print(f'left: {d_left}, right: {d_right}, x: {x}, y: {y}')

    #         # Update plot
    #         scat.set_data(x, y)
    #     except (ValueError, IndexError):
    #         pass  # Handle any parsing errors

    return scat,

# Set up animation
ani = FuncAnimation(fig, update, interval=0.1)  # Update every 100 ms

plt.show()

# Close serial port when done
ser.close()
