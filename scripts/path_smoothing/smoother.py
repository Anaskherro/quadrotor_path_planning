import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate as interp

def interpolate_polyline(polyline, num_points):
    duplicates = []
    for i in range(1, len(polyline)):
        if np.allclose(polyline[i], polyline[i-1]):
            duplicates.append(i)
    if duplicates:
        polyline = np.delete(polyline, duplicates, axis=0)
    tck, u = interp.splprep(polyline.T, s=0)
    u = np.linspace(0.0, 1.0, num_points)
    return np.column_stack(interp.splev(u, tck))

# Read data from path2.txt
with open("path2.txt", "r") as file:
    lines = file.readlines()

# Convert data to numpy array
polyline = np.array([list(map(float, line.strip().split(','))) for line in lines])

# Call interpolate_polyline function
num_points = 100  # Change this to the desired number of points
result = interpolate_polyline(polyline, num_points)

# Plotting
plt.figure(figsize=(8, 6))
plt.plot(polyline[:, 0], polyline[:, 1], 'ro', label='Original Polyline')
plt.plot(result[:, 0], result[:, 1], 'b-', label='Interpolated Polyline')
plt.title('Original and Interpolated Polylines')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.show()
