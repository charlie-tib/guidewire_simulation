import numpy as np
import csv
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.spatial import ConvexHull
import os

def load_data(filename):
    if not os.path.exists(filename):
        print(f"Warning: Data file {filename} not found.")
        return None
    pts = []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        next(reader) # skip header
        for row in reader:
            if len(row) >= 8:
                # SOFA saves Orientation Quaternions as [Qx, Qy, Qz, Qw] in columns 4 to 7
                pts.append([float(row[4]), float(row[5]), float(row[6]), float(row[7])])
    return np.array(pts)

def plot_orientations(ax, quat_array, color, label_prefix):
    if quat_array is None or len(quat_array) < 4:
        return
        
    # Convert quaternions to rotation matrices
    rot = R.from_quat(quat_array) 
    
    # In our simulation, the guidewire naturally points along the Z-axis: [0, 0, 1]
    v_base = np.array([0, 0, 1])
    v_rotated = rot.apply(v_base)
    
    label_str = f'{label_prefix} Directions'
    # Evaluate Dexterity by checking how much of the sphere surface it covers
    try:
        hull = ConvexHull(v_rotated)
        area = hull.area
        max_area = 4 * np.pi  # ~12.57
        coverage_pct = (area / max_area) * 100
        
        label_str = f'{label_prefix}: {coverage_pct:.2f}% Coverage ({area:.2f} sr)'
        print(f"[{label_prefix}] Orientation Coverage Solid Angle: {area:.2f} steradians ({coverage_pct:.2f}% of max)")
    except Exception as e:
        pass

    # Plot these directional vectors as points on a Unit Sphere
    ax.scatter(v_rotated[:, 0], v_rotated[:, 1], v_rotated[:, 2], color=color, s=20, alpha=0.6, label=label_str)

# Load datasets
q_1mag = load_data("workspace_results_1mag.csv")
q_3mag = load_data("workspace_results_3mag.csv")
q_3mag_homo = load_data("workspace_results_4mag.csv")

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Draw a faint wireframe unit sphere for reference (Dexterity Globe)
u, v = np.mgrid[0:2*np.pi:30j, 0:np.pi:15j]
x = np.cos(u)*np.sin(v)
y = np.sin(u)*np.sin(v)
z = np.cos(v)
ax.plot_wireframe(x, y, z, color="gray", alpha=0.1, linewidth=0.5)

# Plot Data
plot_orientations(ax, q_1mag, 'red', '1-Magnet')
plot_orientations(ax, q_3mag, 'blue', '3-Magnets (+1,-1,+1)')
plot_orientations(ax, q_3mag_homo, 'green', '3-Magnets (+1,+1,+1)')

# Formatting
ax.set_box_aspect([1,1,1])
ax.set_xlabel('Direction X')
ax.set_ylabel('Direction Y')
ax.set_zlabel('Direction Z')
ax.set_title("Tip Orientation Dexterity (Unit Sphere Mapping)")
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
plt.legend()
plt.tight_layout()
plt.show()
