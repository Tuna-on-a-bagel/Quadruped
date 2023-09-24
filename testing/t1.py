import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MultipleLocator

# Sample data
x = np.linspace(0, 10, 100)
y = np.sin(x)

fig, ax = plt.subplots()
ax.plot(x, y)

# Major ticks and grid
ax.xaxis.set_major_locator(MultipleLocator(1))  # Major x-ticks spacing of 1
ax.yaxis.set_major_locator(MultipleLocator(0.5))  # Major y-ticks spacing of 0.5

# Minor ticks and grid
ax.xaxis.set_minor_locator(MultipleLocator(0.2))  # Minor x-ticks spacing of 0.2 (creates additional grid lines between major ticks)
ax.yaxis.set_minor_locator(MultipleLocator(0.1))  # Minor y-ticks spacing of 0.1

# Display grid for both major and minor ticks
ax.grid(which='major', linestyle='-', linewidth='0.5', color='black')
ax.grid(which='minor', linestyle=':', linewidth='0.5', color='gray')

plt.show()