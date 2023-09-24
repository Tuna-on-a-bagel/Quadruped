import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Create initial data storage
x_data = np.zeros(100)
y_data = np.zeros(100)

fig, ax = plt.subplots()
ax.set_xlim(0, 100)  # Set x-axis limits
ax.set_ylim(0, 10)   # Set y-axis limits
line, = ax.plot(0, 0)

def read_sensor_data():
    # Placeholder for your actual sensor data reading logic.
    # This just returns a random value as an example.
    return np.random.rand()

def update(frame):
    # Append frame number to x_data
    x_data.pop(0)
    x_data.append(frame)
    
    # Read sensor data and append to y_data
    y_data.pop(0)
    y_data.append(read_sensor_data())
    
    # Adjust the plot data
    line.set_data(x_data, y_data)
    
    # Adjust x-axis limit to fit new data
    #
    #ax.set_xlim(0, frame)
    
    return line,

# Set up the live updating plot
ani = FuncAnimation(plt.gcf(), update, interval=10, frames=np.linspace(0, 100, 100), blit=True)


plt.show()

