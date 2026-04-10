import sys
import os
import Sofa
import numpy as np

# Ensure the mcr_sim_prb package can be found
# We point to the parent directory of mcr_sim_prb
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'mCR_simulator-master/python'))

from splib3.numerics import Quat, Vec3
from scipy.spatial.transform import Rotation as R

# Note: We import from mcr_sim_prb instead of mcr_sim
from mcr_sim_prb import \
    mcr_environment, mcr_instrument, mcr_external_magnet, mcr_simulator, \
    mcr_controller_sofa, mcr_magnet

# === PRB Hardware Parameters ===
outer_diam = 0.002          # 1.0 mm
inner_diam = 0.0            # Solid rod
magnet_remanence = 1.2      # 1.2 T

# Environment mesh
environment_stl = '/home/wen-zheng/guidewire_simulation/mCR_simulator-master/mesh/anatomies/J2-Naviworks.stl'

# --- Define Segments ---
# PRB 1-Magnet structure: [450mm Proximal] + [110mm Soft + 10mm Magnet]
segments = []
# 1. Proximal Hard (Increased to match original 0.5m total length for better visibility/reach)
segments.append({'length': 0.450, 'E': 500e6, 'num_elem': 45, 'is_magnet': False})

# 1 Unit at the tip
# Soft segment (20mm)
segments.append({'length': 0.020, 'E': 250e6, 'num_elem': 2, 'is_magnet': False})
# Single Magnet segment at the tip (using high stiffness to simulate rigid magnet)
segments.append({'length': 0.010, 'E': 1000e6, 'num_elem': 1, 'is_magnet': True, 'polarity': 1.0})

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

from scipy.spatial.transform import Rotation as R

# --- Collision and Penetration Monitor ---
class CollisionMonitor(Sofa.Core.Controller):
    def __init__(self, instrument_mo, start_height=0.0, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.mo = instrument_mo
        self.start_height = start_height
        self.iteration = 0
        self.last_pos = None

    def onAnimateEndEvent(self, event):
        self.iteration += 1
        if self.iteration % 100 == 0:
            # 获取所有节点坐标
            positions = self.mo.position.value
            
            # 1. 检查有没有任何节点掉出“绝对地板” (血管最低点下方)
            # 血管直径大概 2-3cm, 假设最低不超过起点的 -2cm
            for i, p in enumerate(positions):
                if p[1] < (self.start_height - 0.025):
                    print(f"\033[91m[PENETRATION ALERT] Node {i} dropped to Y={p[1]:.4f}m (Start Y={self.start_height:.4f}m). Guidewire has exited the vessel wall!\033[0m")
                    break # 避免刷屏
            
            # 2. 检查尖端是否有突然的“非物理跳变” (跳变通常意味着由于受力过大冲出了碰撞网格)
            tip_pos = np.array(positions[-1][:3])
            if self.last_pos is not None:
                delta = np.linalg.norm(tip_pos - self.last_pos)
                if delta > 0.01: # 100步内移动超过 1cm 极不正常 (可能是弹射穿模)
                    print(f"\033[91m[PENETRATION ALERT] High-speed pop-out detected! Tip moved {delta*1000:.1f}mm in 100 steps.\033[0m")
            self.last_pos = tip_pos

def createScene(root_node):
    ''' Build SOFA scene '''
    # 显式开启重力 (9.81)，并设置摩擦
    simulator = mcr_simulator.Simulator(root_node=root_node, gravity=[0, -9.81, 0], friction_coef=0.1)
    # 将外部永磁铁初始化在血管正上方 10cm 处
    # 注意：真实血管被向下平移了0.45m，所以血管在Y ~ -0.47m 处
    # 我们将磁铁放在 Y = -0.37m 处
    external_magnet = mcr_external_magnet.ExternalMagnet(
        root_node=root_node,
        name='ExternalMagnet',
        init_pos=[0., -0.37, 0.],
        init_rot=[0., 0., 0.]
    )
    
    # --- 横向摆放修正 ---
    # 将整个物理世界 (血管 + 导丝起始点) 绕 X 轴统一旋转 90 度
    r_rot = R.from_euler('x', 90, degrees=True)
    
    # 1. 旋转环境 (Vessel) - 坐标和姿态都要转！
    pos_env_orig = np.array(T_env_sim[:3])
    pos_env_new = r_rot.apply(pos_env_orig).tolist()
    q_env_orig = R.from_quat(T_env_sim[3:7])
    q_env_new = (r_rot * q_env_orig).as_quat().tolist()
    T_env_sim_horizontal = pos_env_new + q_env_new

    # 2. 旋转导丝起始位置与朝向 (Guidewire Start)
    pos_start_orig = np.array(T_start_sim[:3])
    pos_start_new = r_rot.apply(pos_start_orig).tolist()
    q_start_orig = R.from_quat(T_start_sim[3:7])
    
    # 保持导丝与血管之间的相对姿态绝对不变，只是将整个系统转 90 度
    q_mapped_obj = (r_rot * q_start_orig)
    
    # 针对“微向上倾斜”的修正：
    # 增加一个绕局部 Z 轴向下压头的小旋转（比如 -5 到 -10 度）
    # 血管中心线可能并不完美平行于刚开始的位姿，这里做微调
    r_trim = R.from_euler('z', -8, degrees=True)
    q_mapped_obj_trimmed = q_mapped_obj * r_trim
    T_start_sim_rotated = pos_start_new + q_mapped_obj_trimmed.as_quat().tolist()

    environment = mcr_environment.Environment(
        root_node=root_node,
        environment_stl=environment_stl,
        name='aortic_arch',
        T_env_sim=T_env_sim_horizontal,
        flip_normals=True,
        color=[1., 0., 0., 0.3])
        
    # --- 视角调整 ---
    # 增加一个默认相机，从侧面（Y轴负方向或正方向）看过来，方便一启动就能看到全貌
    # position: 相机位置 (在横着的平面上方/下方)
    # lookAt: 看向血管中心
    root_node.addObject('InteractiveCamera', name='camera', position=[0, 0.4, 0], lookAt=[0, 0, 0], orientation=[0.5, -0.5, 0.5, 0.5])

    # 原来的插入方向是沿 Z 轴正向 [0, 0, 1]，跟着整个世界一起旋转 90 度
    main_dir_rotated = r_rot.apply([0, 0, 1]).tolist()

    instrument = mcr_instrument.Instrument(
        name='prb_mcr_1mag',
        root_node=root_node,
        outer_diam=outer_diam,
        inner_diam=inner_diam,
        magnets=magnets_list,
        segments=segments,
        T_start_sim=T_start_sim_rotated,
        main_direction=main_dir_rotated,
        color=[.2, .8, 1., 1.]
    )
    
    # 注入碰撞监控器 (检查是否相比起始高度下落超过 5cm，判断穿透)
    start_y = T_start_sim_rotated[1]
    root_node.addObject(CollisionMonitor(instrument.MO, start_height=start_y, name="CollisionMonitor"))

    controller_sofa = mcr_controller_sofa.ControllerSofa(
        name='ControllerSofa',
        root_node=root_node,
        external_magnet=external_magnet,
        instrument=instrument,
        T_sim_mns=T_sim_mns,
    )
    root_node.addObject(controller_sofa)
    return root_node
