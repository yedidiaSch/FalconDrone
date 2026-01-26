import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Setup Serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

fig, ax = plt.subplots(figsize=(6,6))
plt.subplots_adjust(left=0.1, bottom=0.1)
ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-1.2, 1.2)
ax.set_aspect('equal')
ax.axis('off')

# Draw the Horizon Ring
circle = plt.Circle((0, 0), 1, color='white', fill=False, lw=2)
ax.add_artist(circle)
# Horizon Line
line, = ax.plot([-1, 1], [0, 0], 'g-', lw=4, label='Horizon')
# Center Marker
ax.plot([0], [0], 'ro') 

last_print_time = time.time()

def update(frame):
    global last_print_time
    if ser.in_waiting > 0:
        try:
            line_data = ser.readline().decode('utf-8').strip()
            if ',' in line_data:
                # Print debug info once per second
                current_time = time.time()
                if current_time - last_print_time > 1.0:
                    print(f"Debug: {line_data}")
                    last_print_time = current_time

                roll, pitch = map(float, line_data.split(','))
                
                # Calculate Horizon Angle (Roll) and Height (Pitch)
                angle = np.radians(roll)
                offset = np.sin(np.radians(pitch)) * 0.5
                
                x = np.array([-1, 1])
                y = np.array([offset, offset])
                
                # Rotate the line according to Roll
                x_rot = x * np.cos(angle) - y * np.sin(angle)
                y_rot = x * np.sin(angle) + y * np.cos(angle)
                
                line.set_data(x_rot, y_rot)
        except:
            pass
    return line,

ani = animation.FuncAnimation(fig, update, interval=20, blit=True)
plt.title("FalconDrone - Artificial Horizon")
plt.show()