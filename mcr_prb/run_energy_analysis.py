"""
指标一：能量与曲率分析 (Mechanism Study) - 离线仿真版本
本脚本实现了 Lee et al. (IEEE RA-L 2026) 论文中的势能最小化运动学分析框架。

核心物理逻辑：
1. 均匀磁场模拟：对应论文中的对 Helmholtz 线圈环境，通过 UniformMagController 实现 0-360 度扫描。
2. 静态平衡求解：使用 SOFA 的 StaticSolver (牛顿迭代法) 替代论文中的 fmincon 优化，寻找合力为零（能量极小点）的位姿。
3. 能量计算：
   - 弹性能 (E_elastic): 基于 Cosserat Rod 理论，通过节点间相对转角计算曲率 kappa，再结合 EI 结算。
   - 磁能 (E_magnetic): 计算磁矩 m 与磁场 B 的点积：-m · B。
4. 解除约束：通过将 RestShapeSpringsForceField 设为 0 来模拟自由弯曲状态，不受初始形状束缚。

用法：
  python run_energy_analysis.py --mode 1 (1-Magnet 模式)
  python run_energy_analysis.py --mode 3 (3-Magnets 异向模式)
"""
import sys
import os
import argparse
import numpy as np
import csv
import time

# --- Setup SOFA Path ---
# Assuming SOFA is in the environment (conda sofanew)
import Sofa
import Sofa.Core
import Sofa.Simulation

# Add simulator to path for mcr_sim_prb imports
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mCR_simulator-master/python'))

from mcr_sim_prb import \
    mcr_environment, mcr_instrument, mcr_external_magnet, mcr_simulator, \
    mcr_magnet

# ====================== ForceField for Uniform B ======================
class UniformMagController(Sofa.Core.Controller):
    def __init__(self, instrument, b_intensity, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.instrument = instrument
        self.b_intensity = b_intensity
        self.b_field = np.array([b_intensity, 0, 0])
        self.angle = 0
        
    def set_field_angle(self, degree):
        """对应论文：改变外部磁场方向 theta_B"""
        self.angle = degree
        rad = np.deg2rad(degree)
        self.b_field = np.array([self.b_intensity * np.cos(rad), self.b_intensity * np.sin(rad), 0])
        
    def onAnimateBeginEvent(self, event):
        # We need to apply forces to the ConstantForceField reliably
        positions = self.instrument.MO.position.value
        n_nodes = len(positions)
        
        with self.instrument.CFF.forces.writeable() as forces:
            # Clear previous forces
            for i in range(len(forces)):
                forces[i] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            for i, magnet in enumerate(self.instrument.magnets):
                if isinstance(magnet, mcr_magnet.Magnet):
                    # Node order in MO is base-to-tip (0 to N-1)
                    node_idx = n_nodes - 1 - i
                    if node_idx < 0 or node_idx >= len(forces):
                        continue
                        
                    q = positions[node_idx][3:7]
                    from scipy.spatial.transform import Rotation as R
                    # Ensure quaternion is valid for scipy [x, y, z, w]
                    norm = np.linalg.norm(q)
                    if norm < 1e-6: q = [0, 0, 0, 1]
                    else: q = [x/norm for x in q]
                    
                    rot = R.from_quat([q[0], q[1], q[2], q[3]])
                    m_world = rot.apply([magnet.polarity, 0, 0]) * magnet.dipole_moment
                    torque = np.cross(m_world, self.b_field)
                    
                    # Apply torque
                    forces[node_idx][3] = torque[0]
                    forces[node_idx][4] = torque[1]
                    forces[node_idx][5] = torque[2]
                    
                    if node_idx == n_nodes - 1:
                        print(f"--- Torque Debug (Angle {self.angle}) ---")
                        print(f"  Node {node_idx} Pos: {positions[node_idx][:3]}")
                        print(f"  Node {node_idx} Quat: {positions[node_idx][3:]}")
                        print(f"  M_world: {m_world}, B_field: {self.b_field}")
                        print(f"  Torque: {torque}")
                        print(f"  CFF Forces size: {len(forces)}, MO nodes: {n_nodes}")

# ====================== Scene Builder ======================
def build_segments(mode):
    segments = []
    segments.append({'length': 0.040, 'E': 500e6, 'num_elem': 20, 'is_magnet': False})
    if mode == 3:
        polarities = [-1.0, 1.0, -1.0]
        for i in range(3):
            segments.append({'length': 0.010, 'E': 250e6, 'num_elem': 5, 'is_magnet': False})
            segments.append({'length': 0.005, 'E': 1000e6, 'num_elem': 2, 'is_magnet': True, 'polarity': polarities[i]})
    else:
        segments.append({'length': 0.045, 'E': 250e6, 'num_elem': 20, 'is_magnet': False})
        segments.append({'length': 0.005, 'E': 1000e6, 'num_elem': 2, 'is_magnet': True, 'polarity': -1.0})
    return segments

def build_magnets_list(segments):
    outer_diam = 0.002
    inner_diam = 0.0
    magnet_remanence = 1.2
    total_nodes = sum(seg['num_elem'] for seg in segments) + 1
    magnets_list = [0.0] * total_nodes
    current_node_from_tip = 0
    for seg in reversed(segments):
        if seg['is_magnet']:
            m = mcr_magnet.Magnet(
                length=seg['length'], outer_diam=outer_diam, inner_diam=inner_diam,
                remanence=magnet_remanence, polarity=seg['polarity']
            )
            magnets_list[current_node_from_tip] = m
        current_node_from_tip += seg['num_elem']
    return magnets_list

def createScene(root_node, mode, b_intensity):
    # Plugins
    root_node.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Grid')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.Topology.Mapping')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.MechanicalLoad')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Model')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Projective')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact')
    root_node.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')
    root_node.addObject('RequiredPlugin', name='Sofa.GL.Component.Shader')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear')
    root_node.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')
    root_node.addObject('RequiredPlugin', name='BeamAdapter')

    root_node.addObject('DefaultAnimationLoop')
    
    # Beam setup
    segments = build_segments(mode)
    magnets_list = build_magnets_list(segments)
    instrument = mcr_instrument.Instrument(root_node, magnets_list, name="Guide", 
                                          segments=segments, 
                                          fixed_directions=[0, 0, 0, 0, 0, 0])
    
    # [核心技术点] 对应论文：寻找最小势能状态 (Energy Minimization)
    # 使用 StaticSolver (非线性牛顿法求解合力为0的点) 直接确定每一帧的平衡态位姿
    combined = instrument.InstrumentCombined
    solver = combined.getObject('EulerImplicitSolver')
    if solver:
        combined.removeObject(solver)
    combined.addObject('StaticSolver', name='StaticSolver', newton_iterations=20, printLog=False)
    
    # Instantiate and add the controller
    mag_ctrl = UniformMagController(instrument, b_intensity)
    root_node.addObject(mag_ctrl)

    # UNLOCK: Instead of removing (which causes segfaults), set stiffness to 0
    if hasattr(instrument, 'InstrumentCombined'):
        combined = instrument.InstrumentCombined
        rss = combined.getObject('RestShapeSpringsForceField')
        if rss:
            rss.stiffness = [0.0]
            rss.angularStiffness = [0.0]
        
        pfc = combined.getObject('PartialFixedConstraint')
        if pfc:
            pfc.fixedDirections = [0,0,0,0,0,0] # Release all
            
        print("Unlocked simulation: RSS stiffness set to 0, PFC released.")

    return instrument, mag_ctrl

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=int, default=3)
    parser.add_argument('--intensity', type=float, default=0.025)
    args = parser.parse_args()
    
    root = Sofa.Core.Node("root")
    instrument, mag_ctrl = createScene(root, args.mode, args.intensity)
    Sofa.Simulation.init(root)
    root.dt = 0.01
    
    # Print magnet info
    for i, m in enumerate(instrument.magnets):
        if isinstance(m, mcr_magnet.Magnet):
            print(f"Magnet at index {i}: dipole_moment = {m.dipole_moment}, polarity = {m.polarity}")

    csv_file = f"energy_results_{args.mode}mag.csv"
    if os.path.exists(csv_file): os.remove(csv_file)
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Angle', 'NodeIdx', 'S_mm', 'Curvature', 'ElasticEnergy', 'MagEnergy'])
    
    scan_step = 5
    steps_per_angle = 500
    from scipy.spatial.transform import Rotation as R

    # Pre-calculate magnet data to avoid any weirdness during the simulation loop
    active_magnets = []
    positions_start = instrument.MO.position.value
    n_nodes_total = len(positions_start)
    for i, m in enumerate(instrument.magnets):
        if hasattr(m, 'dipole_moment'):
            node_idx = n_nodes_total - 1 - i
            if 0 <= node_idx < n_nodes_total:
                active_magnets.append({
                    'node_idx': node_idx,
                    'dipole_moment': m.dipole_moment,
                    'polarity': m.polarity
                })
    print(f"Extracted {len(active_magnets)} active magnets for torque application.")

    for angle in range(0, 361, scan_step):
        # Set field
        mag_ctrl.set_field_angle(angle)
        
        # Animate to steady state
        for step_idx in range(steps_per_angle):
            # Directly apply forces to MechanicalObject
            with instrument.MO.force.writeable() as f_data_mo:
                # Clear forces
                for i in range(len(f_data_mo)): 
                    f_data_mo[i] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                
                # Application logic using pre-extracted data
                curr_pos = instrument.MO.position.value
                num_f_mo = len(f_data_mo)
                for mag_info in active_magnets:
                    n_idx = mag_info['node_idx']
                    if n_idx >= num_f_mo: 
                        n_idx = num_f_mo - 1 if n_idx == num_f_mo else -1
                    if n_idx < 0: continue
                        
                    dm = mag_info['dipole_moment']
                    pol = mag_info['polarity']
                    
                    q = curr_pos[n_idx][3:7]
                    from scipy.spatial.transform import Rotation as R
                    norm = np.linalg.norm(q)
                    q = [x/norm for x in q] if norm > 1e-6 else [0,0,0,1]
                    rot = R.from_quat(q)
                    m_world = rot.apply([pol, 0, 0]) * dm
                    torque = np.cross(m_world, mag_ctrl.b_field)
                    f_data_mo[n_idx][3:6] = list(torque)
            
            Sofa.Simulation.animate(root, root.dt.value)
        
        # Capture data
        positions = instrument.MO.position.value
        print(f"Debug [Angle {angle}] Tip Pos: {positions[-1][:3]}")
        n_nodes = len(positions)
        E_soft, E_magnet = 250e6, 1000e6
        I = (np.pi/64) * (0.002**4)
        total_length = 0
        rows = []
        
        for i in range(1, n_nodes-1):
            # Node order index mapping
            node_idx = i
            q1 = positions[node_idx-1][3:7]
            q2 = positions[node_idx][3:7]
            
            # Print first node quat at angle 0 for debug
            if angle == 0 and i == 1:
                print(f"Debug [0deg]: Node 0 Quat = {q1}, Node 1 Quat = {q2}")

            if np.linalg.norm(q1) < 0.5 or np.linalg.norm(q2) < 0.5:
                continue

            mag_info = instrument.magnets[n_nodes - 1 - node_idx]
            is_mag = isinstance(mag_info, mcr_magnet.Magnet)

            p_prev, p_curr = np.array(positions[node_idx-1][:3]), np.array(positions[node_idx][:3])
            ds = np.linalg.norm(p_curr - p_prev)
            q_prev, q_curr = R.from_quat(q1), R.from_quat(q2)
            q_rel = q_prev.inv() * q_curr
            kappa = np.linalg.norm(q_rel.as_rotvec()) / (ds + 1e-9)
            
            # [弹性能计算] 对应论文：E_el = 0.5 * EI * kappa^2 * ds
            E = E_magnet if is_mag else E_soft
            u_el = 0.5 * E * I * (kappa**2) * ds

            # [磁能计算] 对应论文：E_mag = -m · B
            u_mag = 0.0
            if is_mag:
                m_world = q_curr.apply([mag_info.polarity, 0, 0]) * mag_info.dipole_moment
                u_mag = -np.dot(m_world, mag_ctrl.b_field)
            
            total_length += ds
            rows.append([angle, node_idx, total_length*1000, kappa, u_el, u_mag])
        
        with open(csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(rows)
        print(f"Captured angle {angle}°")

if __name__ == "__main__":
    main()
