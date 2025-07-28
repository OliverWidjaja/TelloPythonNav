import matplotlib.pyplot as plt
import numpy as np

# Setup plot
plt.ion()
fig, axs = plt.subplots(2, 2, figsize=(12, 8))
fig.suptitle('Position Tracking Performance')
lines = []
titles = ['X Position (cm)', 'Y Position (cm)', 'Z Position (cm)', 'Yaw (degrees)']

for i, ax in enumerate(axs.flat):
    line1, = ax.plot([], [], 'b-', label='OptiTrack')
    line2, = ax.plot([], [], 'r--', label='Desired')
    ax.set_title(titles[i])
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True)
    lines.append((line1, line2))

# Simulate data
time_data = np.linspace(0, 10, 100)
for t in time_data:
    for i, (line1, line2) in enumerate(lines):
        line1.set_data(time_data[:int(t*10)], np.random.rand(int(t*10)) * 100)  # Simulated data
        line2.set_data(time_data[:int(t*10)], np.random.rand(int(t*10)) * 100)  # Simulated data
        axs.flat[i].relim()
        axs.flat[i].autoscale_view()

    plt.pause(0.1)  # Pause to update the plot

plt.ioff()
plt.show()  # Final show to keep the plot open