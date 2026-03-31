"""
Generate a synthetic S-bend vessel tube as an STL file + centerline waypoints CSV.
The tube consists of two consecutive opposing 120-degree circular arcs joined smoothly,
forming a challenging S-shaped pathway for guidewire navigation testing.

The centerline starts at the origin and moves HORIZONTALLY to the right (+X axis) in the XY-plane.
It then bends downwards (-Y), and then bends upwards (+Y),
forming a continuous S-curve visible natively in an orthographic front-view camera.
"""
import numpy as np
import csv
import struct
import os

# ====================== Parameters ======================
INNER_RADIUS = 0.002       # 2mm inner radius (4mm diameter) - coronary scale
WALL_THICKNESS = 0.0005    # 0.5mm wall
OUTER_RADIUS = INNER_RADIUS + WALL_THICKNESS

# S-bend geometry: two arcs with opposing curvature
ARC_RADIUS = 0.020         # 20mm bend radius
ARC_ANGLE_DEG = 180        # 180-degree U-turn
STRAIGHT_ENTRY = 0.030     # 30mm straight entry section
STRAIGHT_INTERMEDIATE = 0.040  # 40mm straight section between arcs
STRAIGHT_EXIT = 0.030      # 30mm straight exit section

# Mesh resolution
N_CIRC = 16                # Points around circumference
N_ARC = 40                 # Points along each 180-deg arc
N_STRAIGHT_ENTRY = 10      # Points along entry straight
N_STRAIGHT_INTER = 15      # Points along intermediate straight
N_STRAIGHT_EXIT = 10       # Points along exit straight

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
STL_FILE = os.path.join(OUTPUT_DIR, "synthetic_s_bend.stl")
WAYPOINTS_FILE = os.path.join(OUTPUT_DIR, "s_bend_waypoints.csv")


def generate_centerline():
    """Generate the horizontal progressing S-bend centerline in the XY plane."""
    points = []
    
    # --- Section 1: Straight Entry along +X ---
    for i in range(N_STRAIGHT_ENTRY):
        x = (i / N_STRAIGHT_ENTRY) * STRAIGHT_ENTRY
        points.append(np.array([x, 0.0, 0.0]))
    
    p1 = np.array([STRAIGHT_ENTRY, 0.0, 0.0]) # (0.03, 0, 0)
    
    # --- Section 2: First Arc (180-deg CCW U-turn bending UP to +Y) ---
    # Center c1 = p1 + (0, R, 0)
    c1 = p1 + np.array([0.0, ARC_RADIUS, 0.0])
    # Start at p1 (theta = -pi/2), end at p2 (theta = pi/2)
    for i in range(N_ARC):
        theta = -np.pi/2 + (i / N_ARC) * np.pi
        x = c1[0] + ARC_RADIUS * np.cos(theta)
        y = c1[1] + ARC_RADIUS * np.sin(theta)
        points.append(np.array([x, y, 0.0]))
    
    p2 = c1 + np.array([0.0, ARC_RADIUS, 0.0]) # (0.03, 0.04, 0)
    
    # --- Section 3: Intermediate Straight along -X ---
    v_inter = np.array([-1.0, 0.0, 0.0])
    for i in range(1, N_STRAIGHT_INTER + 1):
        pt = p2 + v_inter * (i / N_STRAIGHT_INTER) * STRAIGHT_INTERMEDIATE
        points.append(pt)
        
    p_last = points[-1] # (-0.01, 0.04, 0)
    
    # --- Section 4: Second Arc (180-deg CW U-turn bending FURTHER UP to +80) ---
    # Current pos p_last = (-0.01, 0.04, 0), current tangent -X.
    # To turn UP (RIGHT relative to -X), we need the center to be at p_last + (0, R, 0)
    c2 = p_last + np.array([0.0, ARC_RADIUS, 0.0])
    # Relative to c2, p_last is at theta = -pi/2.
    # We want a CW turn from -pi/2 to -3pi/2 (sweep from -90 back to -270)
    for i in range(1, N_ARC + 1):
        theta = -np.pi/2 - (i / N_ARC) * np.pi
        x = c2[0] + ARC_RADIUS * np.cos(theta)
        y = c2[1] + ARC_RADIUS * np.sin(theta)
        points.append(np.array([x, y, 0.0]))
        
    p3 = points[-1] # (-0.01, 0.08, 0)
    
    # --- Section 5: Straight Exit along +X ---
    v_exit = np.array([1.0, 0.0, 0.0])
    for i in range(1, N_STRAIGHT_EXIT + 1):
        pt = p3 + v_exit * (i / N_STRAIGHT_EXIT) * STRAIGHT_EXIT
        points.append(pt)
        
    return np.array(points)


def generate_tube_mesh(centerline, radius, n_circ):
    """Generate a triangulated tube mesh around the given centerline."""
    n_pts = len(centerline)
    vertices = []
    
    for i in range(n_pts):
        # Compute local frame (tangent, normal, binormal)
        if i == 0:
            tangent = centerline[1] - centerline[0]
        elif i == n_pts - 1:
            tangent = centerline[-1] - centerline[-2]
        else:
            tangent = centerline[i+1] - centerline[i-1]
        tangent = tangent / np.linalg.norm(tangent)
        
        # In XY plane, normal is in XY, binormal is Z
        binormal = np.array([0, 0, 1])
        normal = np.cross(binormal, tangent)
        normal = normal / np.linalg.norm(normal)
        
        # Generate circle of vertices
        for j in range(n_circ):
            theta = 2 * np.pi * j / n_circ
            pt = centerline[i] + radius * (np.cos(theta) * normal + np.sin(theta) * binormal)
            vertices.append(pt)
    
    # Generate triangles
    triangles = []
    for i in range(n_pts - 1):
        for j in range(n_circ):
            j_next = (j + 1) % n_circ
            # Two triangles per quad
            v00 = i * n_circ + j
            v01 = i * n_circ + j_next
            v10 = (i + 1) * n_circ + j
            v11 = (i + 1) * n_circ + j_next
            triangles.append((v00, v10, v01))
            triangles.append((v01, v10, v11))
    
    return np.array(vertices), triangles


def write_binary_stl(filename, vertices, triangles):
    """Write a binary STL file."""
    with open(filename, 'wb') as f:
        # Header (80 bytes)
        f.write(b'\0' * 80)
        # Number of triangles
        f.write(struct.pack('<I', len(triangles)))
        
        for tri in triangles:
            v0, v1, v2 = vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
            # Compute normal
            edge1 = v1 - v0
            edge2 = v2 - v0
            normal = np.cross(edge1, edge2)
            norm_len = np.linalg.norm(normal)
            if norm_len > 0:
                normal = normal / norm_len
            
            # Write normal
            f.write(struct.pack('<fff', *normal))
            # Write vertices
            f.write(struct.pack('<fff', *v0))
            f.write(struct.pack('<fff', *v1))
            f.write(struct.pack('<fff', *v2))
            # Attribute byte count
            f.write(struct.pack('<H', 0))
    
    print(f"STL written: {filename} ({len(triangles)} triangles)")


def write_waypoints(filename, centerline):
    """Write centerline waypoints to CSV."""
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Index', 'X', 'Y', 'Z'])
        for i, pt in enumerate(centerline):
            writer.writerow([i, pt[0], pt[1], pt[2]])
    print(f"Waypoints written: {filename} ({len(centerline)} points)")


if __name__ == "__main__":
    print("Generating XY-Plane S-bend vessel...")
    print(f"  Inner diameter: {INNER_RADIUS*2*1000:.1f} mm")
    print(f"  Arc radius: {ARC_RADIUS*1000:.1f} mm")
    print(f"  Arc angle: {ARC_ANGLE_DEG}°")
    
    centerline = generate_centerline()
    
    # Total length
    total_len = sum(np.linalg.norm(centerline[i+1] - centerline[i]) for i in range(len(centerline)-1))
    print(f"  Total centerline length: {total_len*1000:.1f} mm")
    
    # Generate inner tube (for collision)
    vertices, triangles = generate_tube_mesh(centerline, INNER_RADIUS, N_CIRC)
    write_binary_stl(STL_FILE, vertices, triangles)
    
    # Write waypoints
    write_waypoints(WAYPOINTS_FILE, centerline)
    
    print("Done!")
