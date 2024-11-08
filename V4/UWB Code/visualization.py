
import threading
import serial
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

"""
# https://ieeexplore.ieee.org/document/8419698 
# do calibration with this paper, and perhaps use the spring control mechanism (much easier than the other one
# 

# https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7891540
# particle filter based localization paper & performance analysis (but with large separation between anchors)


#https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9206153
#farming paper with VERY promising results with 3 anchors, but you can select lateral distance 
# https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8593978
Same guy, but with 3 anchor tag and 3 anchor follower
"""
# Initialize serial port (replace 'COM3' with your serial port)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
# sudo chmod 666 /dev/ttyACM0  

# Constants
anchor_distance = 0.5  # Distance between the two anchors in meters

# Function to calculate tag position
def calculate_position(d_left, d_right):
    # Using basic geometry to solve for (x, y)
    x = (d_left**2 - d_right**2 + anchor_distance**2) / (2 * anchor_distance)
    y = math.sqrt(abs(d_left**2 - x**2))  # abs to avoid sqrt of negative values
    
    return x, y


x_data = 0
y_data = 0
def read_data():
    # f = open("liveFeed.dat", "w")
    global x_data, y_data
    while(True):
        line = ser.readline().decode('utf-8').strip()  # Read from serial
        try:
            distances = line.split()
            # print(distances)
            d_left = float(distances[0])
            d_right = float(distances[1])
            x, y = calculate_position(d_left, d_right)

            x_data = x
            y_data = y

            # f.write(f"{x} {y}\r")
            # f.flush()
            # print(f"{x} {y}")
            print(f'left: {d_left}, right: {d_right}, x: {x}, y: {y}')
        except:
            pass
        time.sleep(0.01)


# Set up the plot
fig, ax = plt.subplots()
ax.set_xlim(-3, 3)  # Adjust based on expected coordinates
ax.set_ylim(0, 5)
scat, = ax.plot([], [], 'bo')  # Blue dot for the tag position

# Update function for live plot
def update(frame):
    global x_data, y_data
    scat.set_data(x_data, y_data)

    return scat,

# Set up animation
ani = FuncAnimation(fig, update, interval=0.0)  # Update every 100 ms


data_thread = threading.Thread(target=read_data, daemon=True)
data_thread.start()

plt.show()

# Close serial port when done
ser.close()
