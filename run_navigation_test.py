"""
Metric 3: Automated Blood Vessel Navigation Test
Compares 1-Magnet vs 3-Magnets (+1,-1,+1) on navigability, safety, and efficiency.

Environment variables:
  MODE=1 or 3          (default: 3)
  VESSEL=synthetic or real  (default: synthetic)
  TRIALS=N             (default: 1)

Usage:
  MODE=3 VESSEL=synthetic TRIALS=1 runSofa run_navigation_test.py
"""
import sys
import os
import Sofa
import numpy as np
import csv
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mCR_simulator-master/python'))

from splib3.numerics import Quat, Vec3
from scipy.spatial.transform import Rotation as R

from mcr_sim_prb import \
    mcr_environment, mcr_instrument, mcr_external_magnet, mcr_simulator, \
    mcr_controller_sofa, mcr_magnet, mcr_mag_controller

# ====================== Configuration ======================
MODE = int(os.environ.get("MODE", 3))
VESSEL = os.environ.get("VESSEL", "synthetic")
TRIALS = int(os.environ.get("TRIALS", 1))

# Navigation parameters
INSERTION_SPEED = 0.0002     # m/step: slower for tighter turns
WAYPOINT_REACH_DIST = 0.005  # 5mm reach dist
MAX_DEVIATION = 0.015        # 15mm: fail if tip deviates too far
MAX_STEPS = 150000           
EPM_OFFSET_Z = -0.150        # -150mm: Extreme weak magnetic field test
LOOK_AHEAD_DIST = 0.020      # 20mm: target ahead distance
WARMUP_STEPS = 100           # Wait for physics to settle before checking failure
UPDATE_PERIOD = 1            # Update EPM every step for smoothness
EPM_NOISE_MM = 1.0           # 1mm random noise to simulate control/vibration
# Hardware parameters (same as workspace MC)
outer_diam = 0.002
inner_diam = 0.0
magnet_remanence = 1.2

# ====================== Load Waypoints ======================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

if VESSEL == "synthetic":
    WAYPOINTS_FILE = os.path.join(SCRIPT_DIR, "s_bend_waypoints.csv")
    VESSEL_STL = os.path.join(SCRIPT_DIR, "synthetic_s_bend.stl")
else:
    # For the real aortic arch we need to extract centerline points
    # For now, use a simplified straight-through path
    WAYPOINTS_FILE = os.path.join(SCRIPT_DIR, "aortic_waypoints.csv")
    VESSEL_STL = os.path.join(SCRIPT_DIR, "mCR_simulator-master/mesh/anatomies/J2-Naviworks.stl")

def load_waypoints(filepath):
    """Load waypoint CSV: Index, X, Y, Z (in meters)"""
    pts = []
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # Skip header
        for row in reader:
            pts.append([float(row[1]), float(row[2]), float(row[3])])
    return np.array(pts)

# ====================== Build Guidewire Segments ======================
def build_segments(mode):
    segments = []
    segments.append({'length': 0.450, 'E': 500e6, 'num_elem': 45, 'is_magnet': False})
    
    if mode == 3:
        polarities = [-1.0, 1.0, -1.0]
        for i in range(3):
            segments.append({'length': 0.020, 'E': 250e6, 'num_elem': 2, 'is_magnet': False})
            segments.append({'length': 0.010, 'E': 1000e6, 'num_elem': 1, 'is_magnet': True, 'polarity': polarities[i]})
    elif mode == 1:
        segments.append({'length': 0.060, 'E': 500e6, 'num_elem': 6, 'is_magnet': False})
        segments.append({'length': 0.020, 'E': 250e6, 'num_elem': 2, 'is_magnet': False})
        segments.append({'length': 0.010, 'E': 1000e6, 'num_elem': 1, 'is_magnet': True, 'polarity': -1.0})
    
    return segments

def build_magnets_list(segments):
    total_nodes = sum(seg['num_elem'] for seg in segments) + 1
    magnets_list = [0.0] * total_nodes
    current_node_from_tip = 0
    for seg in reversed(segments):
        if seg['is_magnet']:
            m = mcr_magnet.Magnet(
                length=seg['length'],
                outer_diam=outer_diam,
                inner_diam=inner_diam,
                remanence=magnet_remanence,
                polarity=seg['polarity']
            )
            magnets_list[current_node_from_tip] = m
        current_node_from_tip += seg['num_elem']
    return magnets_list

# ====================== Navigation Controller ======================
class NavigationController(Sofa.Core.Controller):
    """
    Automated navigation controller that:
    1. Smoothly tracks the centerline using a look-ahead point
    2. Incrementally inserts the guidewire
    3. Records contact forces, tip position, path deviation
    4. Supports Manual Override (Press 'M' to toggle)
    """
    
    def __init__(self, root_node, instrument, epm, mag_controller, 
                 waypoints, mode, trial_id, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.root_node = root_node
        self.instrument = instrument
        self.epm = epm
        self.mag_ctrl = mag_controller
        self.waypoints = waypoints
        self.mode = mode
        self.trial_id = trial_id
        
        self.step_cnt = 0
        self.current_wp_idx = 0
        self.insertion_amount = 0.030 
        self.finished = False
        self.success = False
        self.manual_mode = False
        
        self.peak_contact_force = 0.0
        # New Metric: Per-waypoint minimum distance (Path-based RMSE)
        self.wp_min_distances = np.full(len(self.waypoints), 1e9)
        self.look_ahead_idx = 0
        
        # Seed randomness with trial_id for reproducible variance
        np.random.seed(int(self.trial_id))
        
        # CSV output
        mode_str = f"{mode}mag"
        vessel_str = VESSEL
        self.csv_file = f"nav_results_{mode_str}_{vessel_str}_trial{trial_id}.csv"
        
        # Write header
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Step', 'Tip_X', 'Tip_Y', 'Tip_Z', 
                           'Target_WP', 'Deviation_mm', 'ContactForce_mN',
                           'Insertion_mm', 'Status'])
        
        print(f"\n=== Navigation [SMOOTH]: MODE={mode}, VESSEL={VESSEL}, Trial={trial_id} ===")
        print("  Controls: [M] Toggle Auto/Manual | [Up/Down] Manual Insertion (if manual)")
        
    def _get_tip_position(self):
        positions = self.instrument.MO.position.value
        return np.array(positions[-1][:3])
    
    def _find_nearest_index(self, tip_pos):
        """Find the index of the waypoint closest to the tip, searching only forward in a tight window."""
        # Tight window (10 points ~ 15-20mm) prevents jumping across U-turns
        search_range = 10 
        start = self.current_wp_idx
        end = min(len(self.waypoints), start + search_range)
        
        relevant_wps = self.waypoints[start:end]
        if len(relevant_wps) == 0:
            return self.current_wp_idx
            
        dists = np.linalg.norm(relevant_wps - tip_pos, axis=1)
        local_idx = np.argmin(dists)
        min_dist = dists[local_idx]
        
        # Only advance index if we are reasonably close to the waypoint (prevent jumping through walls)
        if min_dist < 0.015: # 15mm threshold
            new_idx = start + local_idx
            self.current_wp_idx = max(self.current_wp_idx, new_idx)
            
        return self.current_wp_idx

    def _get_deviation(self, tip_pos, nearest_idx):
        """Get distance to the centerline segment at nearest index."""
        p1 = self.waypoints[nearest_idx]
        return np.linalg.norm(tip_pos - p1)
    
    def _estimate_contact_force(self):
        velocities = self.instrument.MO.velocity.value
        if velocities is not None and len(velocities) > 0:
            tip_vel = np.array(velocities[-1][:3])
            # Rough proxy for contact vibration/reaction
            return np.linalg.norm(tip_vel) * 300.0
        return 0.0
    
    def onKeypressedEvent(self, event):
        key = event['key']
        if key == 'M':
            self.manual_mode = not self.manual_mode
            status = "MANUAL" if self.manual_mode else "AUTOMATIC"
            print(f"\n  [Mode Switch] Current Mode: {status}")
            return

        if not self.manual_mode:
            return

        # --- Manual Controls (only active when self.manual_mode is True) ---
        d_pos = 0.002
        d_angle = 3. * np.pi / 180
        d_ins = 0.001

        # EPM Translation
        if key == 'W': self.epm.translate_magnet([0, d_pos, 0])
        elif key == 'S': self.epm.translate_magnet([0, -d_pos, 0])
        elif key == 'A': self.epm.translate_magnet([-d_pos, 0, 0])
        elif key == 'D': self.epm.translate_magnet([d_pos, 0, 0])
        elif key == 'Q': self.epm.translate_magnet([0, 0, -d_pos])
        elif key == 'E': self.epm.translate_magnet([0, 0, d_pos])

        # EPM Rotation
        elif key == 'J': self.epm.rotate_magnet([0, 0, 1], d_angle)
        elif key == 'L': self.epm.rotate_magnet([0, 0, 1], -d_angle)
        elif key == 'I': self.epm.rotate_magnet([1, 0, 0], -d_angle)
        elif key == 'K': self.epm.rotate_magnet([1, 0, 0], d_angle)

        # Insertion
        elif key == 'uparrow':
            self.insertion_amount += d_ins
            self.instrument.IRC.xtip.value = [self.insertion_amount]
        elif key == 'downarrow':
            self.insertion_amount -= d_ins
            self.instrument.IRC.xtip.value = [self.insertion_amount]

        # Update physical pose for visualization
        q_rot = R.from_matrix(self.epm.m_epm_dir_matrix).as_quat().tolist() if hasattr(self.epm, 'm_epm_dir_matrix') else [0,0,0,1]
        # Fallback to current dir if matrix not available
        self.epm.mech_obj.position.value = [self.epm.pos.tolist() + q_rot]
            
    def _smooth_nav_logic(self):
        """Dynamic look-ahead navigation."""
        tip_pos = self._get_tip_position()
        curr_idx = self._find_nearest_index(tip_pos)
        
        # Target a point ~15mm ahead on the centerline
        # Find index that is roughly LOOK_AHEAD_DIST away from curr_idx
        # (Assuming waypoints are ~1.5mm apart in synthetic bend)
        ahead_steps = int(LOOK_AHEAD_DIST / 0.0015) 
        target_idx = min(len(self.waypoints) - 1, curr_idx + ahead_steps)
        self.look_ahead_idx = target_idx
        
        # Calculate EPM position with noise
        self._set_epm_for_waypoint(target_idx)

    def _set_epm_for_waypoint(self, wp_idx):
        target_wp = self.waypoints[wp_idx]
        
        # Add random noise to simulate real-world control errors
        noise = np.random.normal(0, EPM_NOISE_MM / 1000.0, size=3)
        noisy_target = target_wp + noise
        
        # Position EPM relative to noisy target (Ignore Z noise for offset consistency)
        epm_pos = noisy_target + np.array([0, 0, EPM_OFFSET_Z])
        
        # EPM Direction: Purely towards the target point
        epm_dir = target_wp - epm_pos
        epm_dir = epm_dir / (np.linalg.norm(epm_dir) + 1e-12)
        
        # Update EPM Orientation
        z_axis = np.array([0, 0, 1]) 
        v_rot = np.cross(z_axis, epm_dir)
        if np.linalg.norm(v_rot) < 1e-9:
             q_rot = [0, 0, 0, 1] if epm_dir[2] > 0 else [0, 1, 0, 0]
        else:
             v_rot /= np.linalg.norm(v_rot)
             half_angle = np.arccos(np.clip(np.dot(z_axis, epm_dir), -1.0, 1.0)) / 2.0
             q_rot = [v_rot[0]*np.sin(half_angle), v_rot[1]*np.sin(half_angle), v_rot[2]*np.sin(half_angle), np.cos(half_angle)]
        
        self.epm.mech_obj.position.value = [epm_pos.tolist() + q_rot]
        
        # Control Mag Controller directly
        self.epm.pos = epm_pos
        self.epm.m_epm_dir = epm_dir
        self.epm.m_epm = epm_dir * self.epm.m_epm_magnitude

    def onAnimateBeginEvent(self, event):
        if self.finished: return
        self.step_cnt += 1
        
        if self.step_cnt % UPDATE_PERIOD != 0: return

        tip_pos = self._get_tip_position()
        curr_idx = self._find_nearest_index(tip_pos)
        deviation = self._get_deviation(tip_pos, curr_idx)
        
        if not self.manual_mode:
            self._smooth_nav_logic()
            # Constantly push forward
            self.insertion_amount += INSERTION_SPEED
            self.instrument.IRC.xtip.value = [self.insertion_amount]
        
        # Metrics Tracking
        curr_force = self._estimate_contact_force()
        self.peak_contact_force = max(self.peak_contact_force, curr_force)
        
        # Update path coverage metric: for waypoints near the tip, record the closest we've ever been
        # Search window around curr_idx
        search_range = 5
        start_w = max(0, curr_idx - search_range)
        end_w = min(len(self.waypoints), curr_idx + search_range)
        for i in range(start_w, end_w):
            # NEW: Calculate deviation ONLY in XY-plane to ignore wall-riding height
            diff = tip_pos - self.waypoints[i]
            d_xy = np.linalg.norm(diff[:2])
            if d_xy < self.wp_min_distances[i]:
                self.wp_min_distances[i] = d_xy
        
        # Logging/Checking
        if self.step_cnt % 500 == 0:
            print(f"  [Step {self.step_cnt}] WP_Progress: {curr_idx}/{len(self.waypoints)}, Dev: {deviation*1000:.1f}mm")
            
        # Success check
        # Must be at the last few waypoints AND physically close to the final exit point
        if curr_idx >= len(self.waypoints) - 3:
            dist_to_end = np.linalg.norm(tip_pos - self.waypoints[-1])
            if dist_to_end < 0.005: # 5mm from the very end
                self.success = True
                self.finished = True
                print("\n  ✅ SUCCESS: Reached final vessel exit!")
            
        # Failure check
        # Only check after warm-up steps to allow the 30mm initial insertion to settle
        if self.step_cnt > WARMUP_STEPS:
            if deviation > MAX_DEVIATION:
                self.success = False
                self.finished = True
                print(f"\n  ❌ FAILED: Deviation {deviation*1000:.1f}mm too high at Step {self.step_cnt}")
                print(f"    Tip Pos: {tip_pos*1000} mm | WP[{curr_idx}]: {self.waypoints[curr_idx]*1000} mm")
            
            if self.step_cnt > MAX_STEPS:
                self.success = False
                self.finished = True
                print("\n  ❌ FAILED: Timeout")

        # Record Data
        if self.step_cnt % 100 == 0:
            with open(self.csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([self.step_cnt, tip_pos[0], tip_pos[1], tip_pos[2],
                               curr_idx, deviation*1000, curr_force, self.insertion_amount*1000, "active"])

        if self.finished:
            print(f"  Simulation Terminated. Success={self.success}")
            self.root_node.animate = False
            # Write summary
            summary_file = f"nav_summary_{self.mode}mag_{VESSEL}.csv"
            header = ['Trial', 'Mode', 'Vessel', 'Success', 'Steps', 'WP_Reached', 'WP_Total', 'PeakForce_mN', 'CumForceImpulse_mNs', 'RMSE_mm']
            file_exists = os.path.exists(summary_file)
            with open(summary_file, 'a', newline='') as f:
                writer = csv.writer(f)
                if not file_exists:
                    writer.writerow(header)
                # Calculate Path-based RMSE (only for waypoints we've reached or passed)
                # We consider waypoints up to curr_idx (the furthest point reached)
                relevant_dists = self.wp_min_distances[:curr_idx+1]
                # Filter out waypoints that were never reached (still 1e9)
                valid_mask = relevant_dists < 1e8
                if np.any(valid_mask):
                    rmse = np.sqrt(np.mean(relevant_dists[valid_mask]**2)) * 1000.0 # to mm
                else:
                    rmse = deviation * 1000.0
                    
                writer.writerow([self.trial_id, self.mode, VESSEL, 1 if self.success else 0, self.step_cnt, curr_idx, len(self.waypoints), self.peak_contact_force, 0, rmse])
            sys.exit(0)


# ====================== SOFA Scene ======================
def createScene(root_node):
    # Load waypoints
    waypoints = load_waypoints(WAYPOINTS_FILE)
    print(f"Loaded {len(waypoints)} waypoints from {WAYPOINTS_FILE}")
    
    # Build guidewire
    segments = build_segments(MODE)
    magnets_list = build_magnets_list(segments)
    
    # Simulator: no gravity for synthetic vessel (pure navigation test),
    # gravity enabled for real vessel (clinical realism)
    if MODE == 3:
        simulator = mcr_simulator.Simulator(root_node=root_node, gravity=[0, 0, 9.81], friction_coef=0.5)
    else:
        simulator = mcr_simulator.Simulator(root_node=root_node, gravity=[0, 0, 9.81], friction_coef=0.5)
    
    # External Permanent Magnet - start above the first waypoint (-Z)
    init_epm_pos = waypoints[0] + np.array([0, 0, EPM_OFFSET_Z])
    external_magnet = mcr_external_magnet.ExternalMagnet(
        root_node=root_node,
        name='ExternalMagnet',
        init_pos=init_epm_pos.tolist(),
        init_rot=[0., 0., 0.]
    )
    
    # Vessel Environment
    if VESSEL == "synthetic":
        # The synthetic vessel lies in the XY plane, starting at origin, pointing +X
        # Guidewire naturally maps its 1D length along 'main_direction'. 
        # Identity quaternion [0,0,0,1] ensures it points exactly along +X without side effects.
        T_start_sim = [waypoints[0][0], waypoints[0][1], waypoints[0][2], 0., 0., 0., 1.]
        main_direction = [1, 0, 0]  # +X entry
        
        # Load STL vessel
        vessel = root_node.addChild('Vessel')
        vessel.addObject('MeshSTLLoader', name='loader', filename=VESSEL_STL)
        vessel.addObject('MeshTopology', src='@loader')
        vessel.addObject('MechanicalObject', name='vesselMO')
        vessel.addObject('TriangleCollisionModel', moving=False, simulated=False, group=2)
        vessel.addObject('LineCollisionModel', moving=False, simulated=False, group=2)
        vessel.addObject('PointCollisionModel', moving=False, simulated=False, group=2)
        
        # Visual model
        vessel_visu = vessel.addChild('Visual')
        vessel_visu.addObject('OglModel', name='vesselOgl', src='@../loader',
                            color=[0.8, 0.2, 0.2, 0.3])
        vessel_visu.addObject('IdentityMapping')
    else:
        # Real aortic arch - use the existing environment setup from run_prb_mcr_sofa.py
        environment_stl = VESSEL_STL
        transl_env_sim = [0., -0.45, 0.]
        rot_env_sim = [-0.7071068, 0, 0, 0.7071068]
        T_env_sim = transl_env_sim + rot_env_sim
        
        r_rot = R.from_euler('x', 90, degrees=True)
        pos_env_orig = np.array(T_env_sim[:3])
        pos_env_new = r_rot.apply(pos_env_orig).tolist()
        q_env_orig = R.from_quat(T_env_sim[3:7])
        q_env_new = (r_rot * q_env_orig).as_quat().tolist()
        T_env_sim_horizontal = pos_env_new + q_env_new
        
        environment = mcr_environment.Environment(
            root_node=root_node,
            environment_stl=environment_stl,
            name='aortic_arch',
            T_env_sim=T_env_sim_horizontal,
            flip_normals=True,
            color=[1., 0., 0., 0.3])
        
        T_start_env = [-.075, -.001, -.020, 0., -0.3826834, 0., 0.9238795]
        trans_start_env = Vec3(T_start_env[0], T_start_env[1], T_start_env[2])
        r_env = R.from_quat(rot_env_sim)
        trans_start_env = r_env.apply(trans_start_env)
        quat_start = Quat(rot_env_sim)
        qrot = Quat(T_start_env[3], T_start_env[4], T_start_env[5], T_start_env[6])
        quat_start.rotateFromQuat(qrot)
        
        T_start_sim_orig = [
            trans_start_env[0]+transl_env_sim[0],
            trans_start_env[1]+transl_env_sim[1],
            trans_start_env[2]+transl_env_sim[2],
            quat_start[0], quat_start[1], quat_start[2], quat_start[3]]
        
        pos_start_orig = np.array(T_start_sim_orig[:3])
        pos_start_new = r_rot.apply(pos_start_orig).tolist()
        q_start_orig = R.from_quat(T_start_sim_orig[3:7])
        q_mapped_obj = (r_rot * q_start_orig)
        r_trim = R.from_euler('z', -8, degrees=True)
        q_mapped_obj_trimmed = q_mapped_obj * r_trim
        T_start_sim = pos_start_new + q_mapped_obj_trimmed.as_quat().tolist()
        
        main_dir_rotated = r_rot.apply([0, 0, 1]).tolist()
        main_direction = main_dir_rotated
    
    # Instrument
    instrument = mcr_instrument.Instrument(
        name=f'nav_mcr_{MODE}mag',
        root_node=root_node,
        outer_diam=outer_diam,
        inner_diam=inner_diam,
        magnets=magnets_list,
        segments=segments,
        T_start_sim=T_start_sim,
        main_direction=main_direction,
        color=[.2, .8, 1., 1.] if MODE == 3 else [1., .5, .2, 1.]
    )
    
    # Initially insert 30mm (already past the tube entry)
    instrument.IRC.xtip.value = [0.030]
    
    # Magnetic controller
    T_sim_mns = [0., 0., 0., 0., 0., 0., 1]
    mag_ctrl = mcr_mag_controller.MagController(
        name='mag_controller',
        external_magnet=external_magnet,
        instrument=instrument,
        T_sim_mns=T_sim_mns,
    )
    root_node.addObject(mag_ctrl)
    
    # Navigation controller
    trial_id = int(os.environ.get("TRIAL_ID", 1))
    nav_ctrl = NavigationController(
        root_node, instrument, external_magnet, mag_ctrl,
        waypoints, MODE, trial_id, name="NavCtrl"
    )
    root_node.addObject(nav_ctrl)
    
    # Camera
    cam_center = waypoints[len(waypoints)//2]
    root_node.addObject('InteractiveCamera', name='camera',
                       position=[cam_center[0], cam_center[1] + 0.15, cam_center[2]],
                       lookAt=cam_center.tolist())
    
    return root_node
