import numpy as np
import csv
import matplotlib.pyplot as plt
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
            if len(row) >= 4:
                pts.append([float(row[1]), float(row[2]), float(row[3])])
    return np.array(pts)

# Load datasets
df_3mag = load_data("workspace_results_3mag.csv")
df_1mag = load_data("workspace_results_1mag.csv")
df_3mag_homo = load_data("workspace_results_4mag.csv")

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

def plot_workspace(pts_array, color, label_prefix):
    if pts_array is None or len(pts_array) < 4:
        return
        
    pts = pts_array * 1000 # Convert to mm
    
    try:
        hull = ConvexHull(pts)
        vol = hull.volume / 1000.0 # cm^3
        
        # Max theoretical reachable volume: Sphere of R=110mm
        max_vol_mm3 = (4.0/3.0) * np.pi * (110**3)
        max_vol_cm3 = max_vol_mm3 / 1000.0
        pct = (vol / max_vol_cm3) * 100.0
        
        label = f"{label_prefix}: {vol:.2f} cm³ ({pct:.2f}% cov)"
        
        # Plot points
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=color, s=2, alpha=0.5, label=label)
        
        # Plot convex hull surfaces
        for simplex in hull.simplices:
            ax.plot(pts[simplex, 0], pts[simplex, 1], pts[simplex, 2], '-', c=color, alpha=0.3, linewidth=0.5)
            
    except Exception as e:
        print(f"Could not compute Convex Hull for {label_prefix}: {e}")
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=color, s=2, alpha=0.5, label=f"{label_prefix} Workspace")

if df_1mag is not None:
    plot_workspace(df_1mag, 'red', '1-Magnet')

if df_3mag is not None:
    plot_workspace(df_3mag, 'blue', '3-Magnets (+1,-1,+1)')
    
if df_3mag_homo is not None:
    plot_workspace(df_3mag_homo, 'green', '3-Magnets (+1,+1,+1)')

# Also plot the origin point (Anchor base is at [0,0,0], tip starts at Z=110mm)
ax.scatter([0], [0], [0], color='black', marker='X', s=50, label='Base Mount (0,0,0)')
ax.scatter([0], [0], [110], color='black', marker='^', s=50, label='Neutral Tip (Z=110mm)')

ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
ax.set_title('3D Reachable Workspace Comparison\\n(Monte Carlo Simulation R_EPM=100mm)')
ax.legend()
plt.tight_layout()
plt.show()
