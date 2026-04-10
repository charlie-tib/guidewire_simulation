import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def get_hull_points(df):
    pts = []
    for node in ['p20', 'p50', 'p80', 'tip']:
        pts.append(df[[f'{node}_x', f'{node}_y', f'{node}_z']].values * 1000)
    return np.vstack(pts)

def calculate_hull(pts):
    try:
        hull = ConvexHull(pts)
        return hull
    except:
        return None

def plot_hull(ax, pts, hull, color, label, alpha=0.1):
    if hull is None: return None
    for s in hull.simplices:
        tri = Poly3DCollection([pts[s]], alpha=alpha, facecolor=color, edgecolor=color, linewidths=0.1)
        ax.add_collection3d(tri)
    return plt.Rectangle((0, 0), 1, 1, fc=color, alpha=0.3, label=label)

def plot_workspace_comparison():
    files = {
        '3-Mag (+-+)': 'python_workspace_results_3mag.csv',
        '3-Mag (+++)': 'python_workspace_results_3mag_pos.csv',
        'Fair Ctrl': 'python_workspace_results_fair.csv',
        'Standard': 'python_workspace_results_1mag.csv'
    }
    colors = {
        '3-Mag (+-+)': 'red',
        '3-Mag (+++)': 'orange',
        'Fair Ctrl': 'green',
        'Standard': 'blue'
    }
    
    dfs = {}
    for label, f in files.items():
        try:
            dfs[label] = pd.read_csv(f)
        except FileNotFoundError:
            print(f"File {f} not found, skipping...")

    if not dfs: return

    fig = plt.figure(figsize=(24, 8))
    v_ref = (2/3) * np.pi * (110**3)

    # 1. 3D Workspace Comparison
    ax1 = fig.add_subplot(131, projection='3d')
    handles = []
    for label, df in dfs.items():
        pts = get_hull_points(df)
        hull = calculate_hull(pts)
        vol_p = (hull.volume / v_ref * 100) if hull else 0
        
        # 绘制点云
        ax1.scatter(df['tip_x']*1000, df['tip_y']*1000, df['tip_z']*1000, c=colors[label], s=2, alpha=0.1)
        
        # 绘制凸包
        p = plot_hull(ax1, pts, hull, colors[label], f"{label} ({vol_p:.1f}%)", alpha=0.1)
        if p: handles.append(p)

    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Y (mm)')
    ax1.set_zlabel('Z (mm)')
    ax1.set_title('3D Workspace Envelope (Global EPM Space)')
    ax1.legend(handles=handles, loc='lower left', fontsize='small')

    # 2. Sigma Theta Distribution
    ax2 = fig.add_subplot(132)
    for label, df in dfs.items():
        ax2.hist(df['sigma_theta'], bins=30, color=colors[label], alpha=0.3, label=label, density=True)
    ax2.set_xlabel('Total Bending Angle (rad)')
    ax2.set_ylabel('Density')
    ax2.set_title('Bending Modal Variety')
    ax2.legend()

    # 3. Z-axis Retraction Depth
    ax3 = fig.add_subplot(133)
    data = []
    labels = []
    for label, df in dfs.items():
        reach = (0.110 - df['tip_z']) * 1000
        data.append(reach)
        labels.append(label)
    
    # 重新命名 labels 以匹配 matplotlib 3.9+ 
    ax3.boxplot(data, tick_labels=labels)
    ax3.set_ylabel('Tip Retraction Depth (mm)\n(Larger = Higher Folding Ability)')
    ax3.set_title('Steerability Depth Analysis')

    plt.tight_layout()
    plt.savefig("python_dvs_workspace_analysis.png")
    print("Saved python_dvs_workspace_analysis.png with 4 groups")

if __name__ == "__main__":
    plot_workspace_comparison()
