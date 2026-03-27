import sys
import os
import Sofa
import numpy as np

# Ensure the mcr_sim_prb package can be found
# We point to the parent directory of mcr_sim_prb
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mCR_simulator-master/python'))

from splib3.numerics import Quat, Vec3
from scipy.spatial.transform import Rotation as R

# Note: We import from mcr_sim_prb instead of mcr_sim
from mcr_sim_prb import \
    mcr_environment, mcr_instrument, mcr_emns, mcr_simulator, \
    mcr_controller_sofa, mcr_magnet

# Calibration file for eMNS (Absolute paths)
cal_path = '/home/wen-zheng/guidewire_simulation/Navion_2_Calibration_24-02-2020.yaml'

# === PRB Hardware Parameters ===
outer_diam = 0.001          # 1.0 mm
inner_diam = 0.0            # Solid rod
magnet_remanence = 1.2      # 1.2 T

# Environment mesh
environment_stl = '/home/wen-zheng/guidewire_simulation/mCR_simulator-master/mesh/anatomies/J2-Naviworks.stl'

# --- Define Segments ---
# PRB structure: [50mm Proximal] + 3x [30mm Soft + 10mm Magnet]
segments = []
# 1. Proximal Hard (Increased to match original 0.5m total length for better visibility/reach)
segments.append({'length': 0.450, 'E': 100e6, 'num_elem': 45, 'is_magnet': False})

# 3 Units
polarities = [1.0, -1.0, 1.0]
for i in range(3):
    # Soft segment
    segments.append({'length': 0.030, 'E': 50e6, 'num_elem': 3, 'is_magnet': False})
    # Magnet segment (using high stiffness to simulate rigid magnet)
    segments.append({'length': 0.010, 'E': 100e6, 'num_elem': 1, 'is_magnet': True, 'polarity': polarities[i]})

# --- Setup Magnets List for Controller ---
# Total nodes = sum of num_elem + 1. 
total_nodes = sum(seg['num_elem'] for seg in segments) + 1

magnets_list = [0.0] * total_nodes
current_node_from_tip = 0 # Starting at the tip
# Segments are defined base-to-tip. Reverse for indexing magnets from tip as expected by MagController.
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

# --- Transforms ---
T_sim_mns = [0., 0., 0., 0., 0., 0., 1]
transl_env_sim = [0., -0.45, 0.]
rot_env_sim = [-0.7071068, 0, 0, 0.7071068]
T_env_sim = [transl_env_sim[0], transl_env_sim[1], transl_env_sim[2], rot_env_sim[0], rot_env_sim[1], rot_env_sim[2], rot_env_sim[3]]

# Starting pose in environment frame (As per original script)
T_start_env = [-.075, -.001, -.020, 0., -0.3826834, 0., 0.9238795]

trans_start_env = Vec3(T_start_env[0], T_start_env[1], T_start_env[2])
r_env = R.from_quat(rot_env_sim)
trans_start_env = r_env.apply(trans_start_env)

quat_start = Quat(rot_env_sim)
qrot = Quat(T_start_env[3], T_start_env[4], T_start_env[5], T_start_env[6])
quat_start.rotateFromQuat(qrot)

T_start_sim = [
    trans_start_env[0]+transl_env_sim[0],
    trans_start_env[1]+transl_env_sim[1],
    trans_start_env[2]+transl_env_sim[2],
    quat_start[0], quat_start[1], quat_start[2], quat_start[3]]

print(f"Calculated T_start_sim: {T_start_sim}")

def createScene(root_node):
    ''' Build SOFA scene '''
    simulator = mcr_simulator.Simulator(root_node=root_node)
    navion = mcr_emns.EMNS(name='Navion', calibration_path=cal_path)
    
    environment = mcr_environment.Environment(
        root_node=root_node,
        environment_stl=environment_stl,
        name='aortic_arch',
        T_env_sim=T_env_sim,
        flip_normals=True,
        color=[1., 0., 0., 0.3])

    instrument = mcr_instrument.Instrument(
        name='prb_mcr',
        root_node=root_node,
        outer_diam=outer_diam,
        inner_diam=inner_diam,
        magnets=magnets_list,
        segments=segments,
        T_start_sim=T_start_sim,
        color=[.2, .8, 1., 1.]
    )

    controller_sofa = mcr_controller_sofa.ControllerSofa(
        name='ControllerSofa',
        root_node=root_node,
        e_mns=navion,
        instrument=instrument,
        T_sim_mns=T_sim_mns,
    )
    root_node.addObject(controller_sofa)
    return root_node
