import sys
import os
import Sofa
import numpy as np
import csv
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mCR_simulator-master/python'))

from splib3.numerics import Quat, Vec3
from scipy.spatial.transform import Rotation as R

from mcr_sim_prb import \
    mcr_instrument, mcr_external_magnet, mcr_simulator, mcr_controller_sofa, mcr_magnet

# === Arguments via Environment Variables ===
MODE = int(os.environ.get("MODE", 3))
SAMPLES = int(os.environ.get("SAMPLES", 2000))
WAIT_STEPS = int(os.environ.get("WAIT_STEPS", 400))
R_EPM = 0.100  # 100mm EPM distance

# === Hardware Parameters ===
outer_diam = 0.002
inner_diam = 0.0
magnet_remanence = 1.2

# --- Define Segments ---
segments = []
# 1. Proximal Hard (450mm to match original so IRC works correctly)
segments.append({'length': 0.450, 'E': 500e6, 'num_elem': 45, 'is_magnet': False})

if MODE == 3:
    polarities = [1.0, -1.0, 1.0]
    for i in range(3):
        segments.append({'length': 0.020, 'E': 250e6, 'num_elem': 2, 'is_magnet': False})
        segments.append({'length': 0.010, 'E': 1000e6, 'num_elem': 1, 'is_magnet': True, 'polarity': polarities[i]})
elif MODE == 4:
    # 3-Magnets with Homogeneous polarities (Cooperative Bending for max workspace)
    polarities = [1.0, 1.0, 1.0]
    for i in range(3):
        segments.append({'length': 0.020, 'E': 250e6, 'num_elem': 2, 'is_magnet': False})
        segments.append({'length': 0.010, 'E': 1000e6, 'num_elem': 1, 'is_magnet': True, 'polarity': polarities[i]})
elif MODE == 1:
    # A realistic 1-Magnet clinical structure:
    # It shouldn't be 80mm of floppy jelly. It should be a stiff shaft until the final 30mm active tip section.
    # Total: 450mm original base + 60mm added stiff shaft = 510mm stiff.
    # Then: 20mm soft hinge + 10mm magnet = 30mm steering tip. Total = 540mm.
    # We add 60mm Hard to the front:
    segments.append({'length': 0.060, 'E': 500e6, 'num_elem': 6, 'is_magnet': False})
    # Then the 20mm Soft hinge:
    segments.append({'length': 0.020, 'E': 250e6, 'num_elem': 2, 'is_magnet': False})
    # Then the 10mm Magnet:
    segments.append({'length': 0.010, 'E': 1000e6, 'num_elem': 1, 'is_magnet': True, 'polarity': 1.0})

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

T_start_sim = [0., 0., 0., 0., 0., 0., 1.]  # Base exactly at origin, aligned with Z

# --- Monte Carlo Controller ---
class MonteCarloController(Sofa.Core.Controller):
    def __init__(self, root_node, instrument, epm, total_samples, wait_steps, mode, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.root_node = root_node
        self.instrument = instrument
        self.epm = epm
        self.total_samples = total_samples
        self.wait_steps = wait_steps
        self.mode = mode
        
        self.step_cnt = 0
        self.sample_idx = 0
        self.results = []
        self.csv_filename = f"workspace_results_{self.mode}mag.csv"
        
        # Initialize CSV
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Sample', 'Tip_X', 'Tip_Y', 'Tip_Z', 'Qx', 'Qy', 'Qz', 'Qw'])
            
    def set_epm_pose(self, pos, d_vec):
        self.epm.pos = np.array(pos)
        self.epm.m_epm_dir = np.array(d_vec) / np.linalg.norm(d_vec)
        self.epm.m_epm = self.epm.m_epm_dir * self.epm.m_epm_magnitude
        
        # Update the SOFA mechanical object strictly for visual feedback
        # SOFA position values are returned as read-only memory views, so we copy it to a list:
        curr_pose = self.epm.mech_obj.position.value[0].tolist()
        
        # Calculate a quaternion that points X-axis towards d_vec
        base_x = np.array([1, 0, 0])
        v = np.cross(base_x, d_vec)
        c = np.dot(base_x, d_vec)
        if c > -0.99999:
            q = np.array([v[0], v[1], v[2], 1 + c])
            q = q / np.linalg.norm(q)
            curr_pose[3:7] = q.tolist()
        curr_pose[0:3] = self.epm.pos.tolist()
        self.epm.mech_obj.position.value = [curr_pose]

    def onAnimateBeginEvent(self, event):
        self.step_cnt += 1
        
        if self.step_cnt % self.wait_steps == 0:
            # 1. Record Data
            if self.sample_idx > 0:  # Skip recording on step 0
                tip_pose = self.instrument.MO.position.value[-1]
                self.results.append([self.sample_idx] + tip_pose.tolist())
                # Write incrementally
                with open(self.csv_filename, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([self.sample_idx] + tip_pose.tolist())
                print(f"[{self.mode}-Mag] Sample {self.sample_idx}/{self.total_samples} recorded. Tip Z={tip_pose[2]*1000:.1f}mm")

            # 2. Check Completion
            if self.sample_idx >= self.total_samples:
                print(f"\\n!!! Monte Carlo Simulation Complete !!!\\nResults saved to {self.csv_filename}")
                self.root_node.animate = False
                sys.exit(0)
                return

            # 3. Apply New Random EPM Pose
            # Position: Hemisphere around the BASE [0,0,0], with a large enough radius
            # to guarantee it never physically touches the 110mm long guidewire tip.
            # R = 150mm ensures at least 40mm clearance from the tip at maximum extension.
            R_EPM_SAFE = 0.150 
            
            phi = np.random.uniform(0, 2 * np.pi)
            theta = np.random.uniform(0, np.pi/2) # Only Z > 0 (front hemisphere)
            
            EPM_X = R_EPM_SAFE * np.sin(theta) * np.cos(phi)
            EPM_Y = R_EPM_SAFE * np.sin(theta) * np.sin(phi)
            EPM_Z = R_EPM_SAFE * np.cos(theta)
            
            # Direction: uniform random 3D unit vector
            u = np.random.normal(0, 1, 3)
            d_vec = u / np.linalg.norm(u)
            
            self.set_epm_pose([EPM_X, EPM_Y, EPM_Z], d_vec)
            self.sample_idx += 1

def createScene(root_node):
    # Free Space Simulator
    simulator = mcr_simulator.Simulator(root_node=root_node, gravity=[0, -9.81, 0], friction_coef=0.0)
    
    # EPM
    external_magnet = mcr_external_magnet.ExternalMagnet(
        root_node=root_node,
        name='ExternalMagnet',
        init_pos=[0., 0.1, 0.],
        init_rot=[0., 0., 0.]
    )
    
    # Instrument
    instrument = mcr_instrument.Instrument(
        name=f'prb_mcr_{MODE}mag',
        root_node=root_node,
        outer_diam=outer_diam,
        inner_diam=inner_diam,
        magnets=magnets_list,
        segments=segments,
        T_start_sim=T_start_sim,
        main_direction=[0, 0, 1],
        color=[.2, .8, 1., 1.] if MODE==3 else [1., .5, .2, 1.]
    )
    
    # Set the initial insertion length (amount of guidewire pushed out)
    # If we want 110mm out, we push it by 0.110. (m_ircontroller handles the proximal anchoring).
    instrument.IRC.xtip.value = [0.110]

    # Controller
    T_sim_mns = [0., 0., 0., 0., 0., 0., 1]
    controller_sofa = mcr_controller_sofa.ControllerSofa(
        name='ControllerSofa',
        root_node=root_node,
        external_magnet=external_magnet,
        instrument=instrument,
        T_sim_mns=T_sim_mns,
    )
    root_node.addObject(controller_sofa)
    
    # Monte Carlo Logic
    mc_ctrl = MonteCarloController(root_node, instrument, external_magnet, SAMPLES, WAIT_STEPS, MODE, name="MC_Ctrl")
    root_node.addObject(mc_ctrl)
    
    # Camera
    root_node.addObject('InteractiveCamera', name='camera', position=[0.2, 0.2, 0.1], lookAt=[0, 0, 0.05])
    
    return root_node
