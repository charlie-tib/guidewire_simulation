import sys
import os
import Sofa
import numpy as np
import Sofa.Core
from scipy.spatial.transform import Rotation as R

# 环境对齐
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mCR_simulator-master/python'))
from mcr_sim_prb import mcr_instrument, mcr_simulator, mcr_magnet

class ScanController(Sofa.Core.Controller):
    def __init__(self, instrument, b_intensity=0.030, *args, **kwargs): 
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.instrument = instrument
        self.b_intensity = b_intensity
        self.angle = 0.0
        # 旋转速度放慢 4 倍 (0.25 度 / step)
        self.dt_angle = 0.25 
        self.viz_mo = None 
        
    def onAnimateBeginEvent(self, event):
        # 旋转磁场
        self.angle += self.dt_angle
        if self.angle >= 360: self.angle = 0
        rad = np.deg2rad(self.angle)
        b_field = np.array([self.b_intensity * np.cos(rad), self.b_intensity * np.sin(rad), 0])
        
        # 物理场同步与驱动
        mo = self.instrument.MO
        n_nodes = len(mo.position.value)
        index_mag = self.instrument.index_mag
        
        viz_poses = []
        with self.instrument.CFF.forces.writeable() as cff_forces:
            for m_idx in index_mag:
                magnet = self.instrument.magnets[m_idx]
                node_idx = n_nodes - m_idx - 1
                pos = mo.position.value[node_idx]
                quat = pos[3:7]
                
                # 数值哨兵
                q_norm = np.linalg.norm(quat)
                if q_norm < 1e-6:
                    r_rot = R.identity()
                else:
                    r_rot = R.from_quat(quat / q_norm)
                
                viz_poses.append(pos)
                m_world = r_rot.apply([magnet.polarity, 0, 0]) * magnet.dipole_moment
                torque = np.cross(m_world, b_field)
                cff_forces[m_idx] = [0, 0, 0, torque[0], torque[1], torque[2]]

        if self.viz_mo is not None:
            self.viz_mo.position.value = viz_poses

def createScene(root_node):
    root_node.dt = 0.01
    simulator = mcr_simulator.Simulator(root_node=root_node, gravity=[0, 0, 0], friction_coef=0.0)
    
    MODE = int(os.environ.get("MODE", 3))

    # 设置刚度为 300MPa
    segments = [{'length': 0.450, 'E': 300e6, 'num_elem': 45, 'is_magnet': False}]
    if MODE == 3:
        p = [1.0, -1.0, 1.0]
        for i in range(3):
            segments.append({'length': 0.020, 'E': 150e6, 'num_elem': 2, 'is_magnet': False})
            segments.append({'length': 0.010, 'E': 800e6, 'num_elem': 1, 'is_magnet': True, 'polarity': p[i]})
        segments.append({'length': 0.020, 'E': 150e6, 'num_elem': 2, 'is_magnet': False})
    else:
        segments.append({'length': 0.080, 'E': 300e6, 'num_elem': 8, 'is_magnet': False})
        segments.append({'length': 0.020, 'E': 150e6, 'num_elem': 2, 'is_magnet': False})
        segments.append({'length': 0.010, 'E': 800e6, 'num_elem': 1, 'is_magnet': True, 'polarity': 1.0})
    
    total_nodes = sum(seg['num_elem'] for seg in segments) + 1
    magnets_list = [0.0] * total_nodes
    curr_n = 0
    for seg in reversed(segments):
        if seg['is_magnet']:
            m = mcr_magnet.Magnet(length=seg['length'], outer_diam=0.002, inner_diam=0.0, remanence=1.2, polarity=seg['polarity'])
            magnets_list[curr_n] = m
        curr_n += seg['num_elem']

    instrument = mcr_instrument.Instrument(
        root_node=root_node, magnets=magnets_list, name=f'mcr_viz_{MODE}mag',
        outer_diam=0.002, inner_diam=0.0, segments=segments,
        T_start_sim=[0, 0, 0, 0, 0, 0, 1], main_direction=[0, 0, 1]
    )
    
    # 外部坐标轴 node (位于根目录，避开 BTD 冲突)
    mag_viz = root_node.addChild('ExternalMagnetAxes')
    viz_mo = mag_viz.addObject('MechanicalObject', name='MagDOFs', template='Rigid3d', 
                               showObject=True, showObjectScale=0.04)
    
    # 锁定 110mm 处的物理位置
    instrument.IRC.xtip.value = [0.110]
    rss = instrument.InstrumentCombined.getObject('RestShapeSpringsForceField')
    if rss: 
        rss.stiffness.value = [1e9] # 极高刚度锁死
        rss.angularStiffness.value = [1e9]

    # 控制器
    ctrl = ScanController(instrument, b_intensity=0.030)
    ctrl.viz_mo = viz_mo
    root_node.addObject(ctrl)
    
    root_node.addObject('InteractiveCamera', position=[0.2, 0.2, 0.15], lookAt=[0, 0, 0.1])
    return root_node
