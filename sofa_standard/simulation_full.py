import Sofa
import Sofa.Core
import numpy as np

# ==============================================================================
# 数学工具：四元数 (XYZW) 转旋转矩阵
# ==============================================================================
def quat_to_matrix(q):
    """将 SOFA 的 XYZW 四元数转换为 3x3 旋转矩阵"""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),       1 - 2*(x**2 + y**2)]
    ])

# ==============================================================================
# 核心控制器：针对 v25.12 优化的磁力与力矩分配
# ==============================================================================
class MagneticController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.wire_mo = kwargs.get('wire_mo')
        self.em_mo = kwargs.get('em_mo')
        self.cff = kwargs.get('cff')
        self.mag_map = kwargs.get('mag_map')
        
        # 物理常数 (SI 单位)
        self.mu_0 = 4 * np.pi * 1e-7
        self.m_em_vector = np.array([0.0, 50.0, 0.0]) # 外部磁体磁矩方向与强度 (Am^2)

    def onAnimateBeginEvent(self, event):
        """每帧开始时计算磁场力与力矩"""
        wire_state = self.wire_mo.position.value
        # 获取外部磁体位置 (Rigid3d 的前三个分量)
        em_pos = self.em_mo.position.value[0][:3]
        
        num_nodes = len(wire_state)
        forces = np.zeros((num_nodes, 6))
        
        for idx, m_scalar in self.mag_map.items():
            if idx >= num_nodes: continue
            
            p_i = wire_state[idx][:3]
            quat = wire_state[idx][3:] # XYZW 格式
            
            # 使用四元数旋转局部磁矩到世界坐标
            R = quat_to_matrix(quat)
            # 假设轴向磁化 (沿导丝 Z 轴)
            m_i_global = R @ np.array([0.0, 0.0, m_scalar])
            
            f, tau = self.calc_dipole_wrench(p_i, m_i_global, em_pos, self.m_em_vector)
            
            forces[idx][:3] = f
            forces[idx][3:] = tau
            
        # v25.12 修复：确保 Data 赋值为展平的列表，以兼容位深与向量转换
        self.cff.forces.value = forces.flatten().tolist()

    def calc_dipole_wrench(self, p_i, m_i, p_em, m_em):
        """基于偶极子模型的磁力与磁力矩计算"""
        r_vec = p_i - p_em
        r = np.linalg.norm(r_vec)
        if r < 1e-4: return np.zeros(3), np.zeros(3)
        
        r_hat = r_vec / r
        # 磁感应强度 B (由外部磁体产生)
        B = (self.mu_0 / (4 * np.pi * r**3)) * (3 * np.dot(m_em, r_hat) * r_hat - m_em)
        # 磁力矩 tau = m x B
        tau = np.cross(m_i, B)
        # 磁力 F = grad(m . B)
        F = (3 * self.mu_0 / (4 * np.pi * r**4)) * (
            np.dot(m_i, r_hat) * m_em + 
            np.dot(m_em, r_hat) * m_i + 
            np.dot(m_i, m_em) * r_hat - 
            5 * np.dot(m_i, r_hat) * np.dot(m_em, r_hat) * r_hat
        )
        return F, tau

# ==============================================================================
# SOFA 场景构建
# ==============================================================================
def createScene(rootNode):
    # 加载必要插件
    rootNode.addObject('RequiredPlugin', pluginName=[
        "Sofa.Component.AnimationLoop", 
        "Sofa.Component.Collision.Detection.Algorithm", 
        "Sofa.Component.Collision.Detection.Intersection", 
        "Sofa.Component.Collision.Geometry", 
        "Sofa.Component.Collision.Response.Contact",
        "Sofa.Component.Constraint.Lagrangian.Correction",
        "Sofa.Component.Constraint.Lagrangian.Solver",
        "Sofa.Component.Constraint.Projective",
        "Sofa.Component.Engine.Select",
        "Sofa.Component.LinearSolver.Direct",
        "Sofa.Component.Mapping.Linear",
        "Sofa.Component.Mass",
        "Sofa.Component.MechanicalLoad",
        "Sofa.Component.SolidMechanics.FEM.Elastic",
        "Sofa.Component.SolidMechanics.Spring",
        "Sofa.Component.StateContainer",
        "Sofa.Component.Topology.Container.Constant",
        "Sofa.Component.Topology.Container.Dynamic",
        "Sofa.Component.Visual",
        "Sofa.Component.IO.Mesh",
        "Sofa.Component.ODESolver.Backward",
        "Sofa.GL.Component.Rendering3D",
        "BeamAdapter"
    ])
    
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance=1e-5)
    
    rootNode.addObject('CollisionPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('LocalMinDistance', alarmDistance=0.005, contactDistance=0.001)
    rootNode.addObject('CollisionResponse', response='FrictionContactConstraint')

    rootNode.gravity = [0, 0, 0]
    # 添加视觉样式，方便在 GUI 中查看
    rootNode.addObject('VisualStyle', displayFlags="showBehaviorModels showCollisionModels showVisual showWireframe")

    # 参数配置 (单位: 米/kg/Pa)
    L = 0.1          # 100 mm 导丝长度
    radius = 0.0004  # 0.4 mm 导丝半径
    E = 4e10         # 40 GPa (Nitinol 近似模量)
    G = 1.5e10       # 剪切模量
    N = 40           # 离散节点数
    
    # 磁化分布：将尖端最后 10 个节点设为磁化段 (Am^2)
    # 模拟真实分布式磁力响应
    ACTIVE_MAG_MAP = {i: 0.005 for i in range(N-10, N)}

    # ================= 1. 外部磁体 (External Magnet) =================
    em_node = rootNode.addChild('ExternalMagnet')
    em_mo = em_node.addObject('MechanicalObject', name='MO', template='Rigid3d', 
                               position=[0.02, 0.05, 0.05, 0, 0, 0, 1], showObject=False)
    em_node.addObject('SphereCollisionModel', radius=0.01, simulated=False)
    
    vis_em = em_node.addChild('Visu')
    # 使用 OglModel 直接显示，如果使用了 SphereCollisionModel 且开启了显示位，通常能看到点
    # 在最新版 SOFA 中，如果想可视化球体而没有 MeshSphere，可以使用包含 topology 的其他方式或加载外部模型
    # 这里通过简单点显示作为回退
    vis_em.addObject('MechanicalObject', showObject=False) # 额外添加以防万一
    vis_em.addObject('OglModel', name='VisualModel', color=[0.2, 0.4, 1.0, 1.0])

    # ================= 2. 磁化导丝 (Magnetic Guidewire) =================
    gw_node = rootNode.addChild('Guidewire')
    # 使用隐式欧拉求解器确保稳定性
    gw_node.addObject('EulerImplicitSolver', rayleighStiffness=0.01, rayleighMass=0.01)
    gw_node.addObject('BTDLinearSolver')
    
    # 沿 Z 轴生成初始位置
    positions = [[0, 0, i * (L / (N - 1)), 0, 0, 0, 1] for i in range(N)]
    wire_mo = gw_node.addObject('MechanicalObject', name='dof', template='Rigid3d', position=positions, showObject=False)
    
    # 显式定义拓扑连接
    edges = [[i, i+1] for i in range(N-1)]
    gw_node.addObject('EdgeSetTopologyContainer', name='topo', edges=edges)
    gw_node.addObject('UniformMass', totalMass=0.0003) # 约 0.3g
    
    # Cosserat 梁参数配置 (v25.12 兼容模式)
    gw_node.addObject('RodStraightSection', name='section', 
                                youngModulus=E, poissonRatio=0.33, radius=radius,
                                length=L, nbEdgesCollis=N-1, nbEdgesVisu=N-1)
    
    gw_node.addObject('WireRestShape', name='rest_shape', template='Rigid3d', wireMaterials='@section')
    gw_node.addObject('WireBeamInterpolation', name='Interpol', WireRestShape='@rest_shape')
    gw_node.addObject('AdaptiveBeamForceFieldAndMass', name='BeamFF', massDensity=6500, interpolation='@Interpol')
    
    # 固定导丝底端
    gw_node.addObject('FixedProjectiveConstraint', indices=0)
    
    # 磁力注入接口
    cff = gw_node.addObject('ConstantForceField', name='MagneticForce', forces=[0]*6*N)
    
    # 碰撞模型映射 (Rigid3d -> Vec3d)
    coll_node = gw_node.addChild('Collision')
    coll_node.addObject('MechanicalObject', name='c_dof', template='Vec3d', showObject=False)
    coll_node.addObject('BeamLinearMapping', input='@../dof', output='@c_dof', isMechanical=True)
    coll_node.addObject('SphereCollisionModel', radius=radius, contactFriction=0.1)

    # 挂载 Python 磁力控制器
    gw_node.addObject(MagneticController(name='MagController', 
                                         wire_mo=wire_mo, 
                                         em_mo=em_mo, 
                                         cff=cff, 
                                         mag_map=ACTIVE_MAG_MAP))
    
    # ================= 3. 血管环境 (Vessel Environment) =================
    # 加载由 generate_vessel.py 生成的 STL 模型
    vessel_node = rootNode.addChild('Vessel')
    # 安全检查：如果文件不存在，请先运行 generate_vessel.py
    vessel_path = '/home/wen-zheng/meshes/vessel.stl'
    vessel_node.addObject('MeshSTLLoader', name='loader', filename=vessel_path)
    vessel_node.addObject('MeshTopology', src='@loader')
    vessel_node.addObject('MechanicalObject', template='Vec3d', showObject=False)
    # 血管壁摩擦力设为 0.05 (模拟亲水涂层)
    vessel_node.addObject('TriangleCollisionModel', simulated=False, contactFriction=0.05)
    
    v_vis = vessel_node.addChild('Visu')
    v_vis.addObject('OglModel', src='@../loader', color=[1.0, 0.7, 0.7, 0.3])
    
    # ================= 4. 可视化映射 =================
    vis_gw = gw_node.addChild('Visu')
    vis_gw.addObject('MechanicalObject', name='v_dof', template='Vec3d', showObject=False)
    vis_gw.addObject('BeamLinearMapping', input='@../dof', output='@v_dof', isMechanical=False)
    vis_gw.addObject('OglModel', name='VisualModel', color=[0.3, 0.3, 0.3, 1.0], 
                     lineWidth=3.0) 

    return rootNode