import Sofa
import Sofa.Core
import numpy as np
from splib3.numerics import Quat, Vec3
import os

# ==============================================================================
# 磁体与控制器逻辑 (复现自 mCR_simulator)
# ==============================================================================

class Magnet:
    def __init__(self, length, outer_diam, inner_diam, remanence):
        self.length = length
        self.outer_diam = outer_diam
        self.inner_diam = inner_diam
        self.remanence = remanence
        self.mu_0 = 4 * np.pi * 1e-7
        self.volume = length * np.pi * ((outer_diam/2)**2 - (inner_diam/2)**2)
        # 磁矩大小
        self.dipole_moment = (1.0 / self.mu_0) * remanence * self.volume

class MCRMagController(Sofa.Core.Controller):
    """
    复现自 mcr_mag_controller.py
    应用力矩 T = m x B 到指定节点
    """
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.instrument_mo = kwargs.get('instrument_mo')
        self.cff = kwargs.get('cff')
        self.magnets = kwargs.get('magnets') # 包含 Magnet 对象的列表，长度与节点数一致
        self.field_des = np.array([0.01, 0.0, 0.0]) # 期望磁场 (Tesla)
        
        # 查找有磁体的节点索引
        self.index_mag = [i for i, m in enumerate(self.magnets) if m is not None]

    def onAnimateBeginEvent(self, event):
        positions = self.instrument_mo.position.value
        num_nodes = len(positions)
        
        # 安全检查：如果节点尚未初始化或数量不匹配，跳过本帧
        if num_nodes < len(self.magnets):
            return
        
        # 重置力场
        forces = np.zeros((num_nodes, 6))
        
        for idx in self.index_mag:
            # SOFA 中的 Rigid3d: [x, y, z, qx, qy, qz, qw]
            p = positions[idx]
            quat = p[3:] # XYZW
            
            magnet = self.magnets[idx]
            moment_magnitude = magnet.dipole_moment
            
            # 将局部轴向 (假设为 X 轴) 转换到世界坐标
            q = Quat(quat)
            x_axis_local = Vec3(1.0, 0.0, 0.0)
            m_global = q.rotate(x_axis_local) * moment_magnitude
            
            # 计算力矩 T = m x B
            B_vec = Vec3(self.field_des[0], self.field_des[1], self.field_des[2])
            torque = Vec3()
            torque = Vec3.cross(m_global, B_vec)
            
            forces[idx][3:] = [torque[0], torque[1], torque[2]]
            
        self.cff.forces.value = forces.flatten().tolist()

# ==============================================================================
# 场景构建
# ==============================================================================

def createScene(rootNode):
    # 加载必要插件 (修正 v25.12 兼容性)
    rootNode.addObject('RequiredPlugin', pluginName=[
        "Sofa.Component.AnimationLoop",
        "Sofa.Component.Collision.Detection.Algorithm",
        "Sofa.Component.Collision.Detection.Intersection",
        "Sofa.Component.Collision.Geometry",
        "Sofa.Component.Collision.Response.Contact",
        "Sofa.Component.Constraint.Lagrangian.Correction",
        "Sofa.Component.Constraint.Lagrangian.Solver",
        "Sofa.Component.Constraint.Projective",
        "Sofa.Component.LinearSolver.Direct",
        "Sofa.Component.Mapping.Linear",
        "Sofa.Component.Mass",
        "Sofa.Component.MechanicalLoad",
        "Sofa.Component.StateContainer",
        "Sofa.Component.Topology.Container.Constant",
        "Sofa.Component.Topology.Container.Dynamic",
        "Sofa.Component.Topology.Container.Grid",
        "Sofa.Component.Visual",
        "Sofa.Component.IO.Mesh",
        "Sofa.Component.ODESolver.Backward",
        "Sofa.Component.SolidMechanics.Spring",
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

    # 参数定义
    young_modulus_body = 170e6
    young_modulus_tip = 21e6
    length_body = 0.5
    length_tip = 0.034
    outer_diam = 0.00133
    num_elem_body = 30
    num_elem_tip = 4
    total_nodes = num_elem_body + num_elem_tip + 1

    # 磁体配置
    magnet_params = Magnet(length=4e-3, outer_diam=1.33e-3, inner_diam=0.8e-3, remanence=1.45)
    magnets = [None] * total_nodes
    magnets[-1] = magnet_params
    magnets[-2] = magnet_params

    # --- 拓扑定义与分段 ---
    topoLines = rootNode.addChild('topoLines')
    # 定义近端 (Body) 与 远端 (Tip)
    topoLines.addObject('RodStraightSection', name='BodySection', 
                         youngModulus=75e9, poissonRatio=0.33, radius=outer_diam/2.0, 
                         length=length_body, nbEdgesCollis=num_elem_body, nbEdgesVisu=num_elem_body)
    
    topoLines.addObject('RodStraightSection', name='TipSection', 
                         youngModulus=210e9, poissonRatio=0.33, radius=outer_diam/2.0, 
                         length=length_tip, nbEdgesCollis=num_elem_tip, nbEdgesVisu=num_elem_tip)
    
    # 核心：使用 wireMaterials 关联分段 (v25.12 兼容)
    topoLines.addObject('WireRestShape', name='RestShape', template='Rigid3d',
                         wireMaterials='@BodySection @TipSection')
    
    # 拓扑容器 (必须包含，以满足 IRController 的修改需求)
    topoLines.addObject('EdgeSetTopologyContainer', name='meshLinesGuide')
    topoLines.addObject('EdgeSetTopologyModifier', name='Modifier')
    topoLines.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    topoLines.addObject('MechanicalObject', name='dofTopo2', template='Rigid3d', showObject=False)

    # --- 机械模型 ---
    instrument = rootNode.addChild('mcr')
    instrument.addObject('EulerImplicitSolver', rayleighStiffness=0.2, rayleighMass=0.1)
    instrument.addObject('BTDLinearSolver')
    
    # 必须在同级节点提供拓扑容器 (v25.12 强制要求)
    instrument.addObject('EdgeSetTopologyContainer', name='MeshLines')
    instrument.addObject('EdgeSetTopologyModifier', name='Modifier')
    instrument.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    
    mo = instrument.addObject('MechanicalObject', name='DOFs', template='Rigid3d', showObject=False)
    instrument.addObject('WireBeamInterpolation', name='Interpol', WireRestShape='@../topoLines/RestShape')
    instrument.addObject('AdaptiveBeamForceFieldAndMass', name='BeamFF', massDensity=155.0, interpolation='@Interpol')
    
    # 插入控制器
    instrument.addObject('InterventionalRadiologyController', 
                         name='IRController',
                         instruments='Interpol',
                         startingPos=[0, 0, 0, 0, 0, 0, 1],
                         xtip=[0.1], 
                         step=0.001,
                         template='Rigid3d')
    
    cff = instrument.addObject('ConstantForceField', name='MagForce', forces=[0]*6*total_nodes)
    
    instrument.addObject('LinearSolverConstraintCorrection', wire_optimization='true')
    instrument.addObject('FixedConstraint', indices=0)
    instrument.addObject('RestShapeSpringsForceField', points='@IRController.indexFirstNode', stiffness=1e8, angularStiffness=1e8)

    instrument.addObject(MCRMagController(name='MagController', 
                                          instrument_mo=mo, 
                                          cff=cff, 
                                          magnets=magnets))

    # --- 碰撞模型 ---
    collis = instrument.addChild('collis')
    collis.addObject('MechanicalObject', name='CollisDOFs', template='Vec3d', showObject=False)
    collis.addObject('BeamLinearMapping', input='@../DOFs', output='@CollisDOFs')
    collis.addObject('SphereCollisionModel', radius=outer_diam/2.0)

    # --- 视觉模型 ---
    visu = instrument.addChild('visu')
    visu.addObject('MechanicalObject', name='VisuDOFs', template='Vec3d', showObject=False)
    visu.addObject('BeamLinearMapping', input='@../DOFs', output='@VisuDOFs')
    visu.addObject('OglModel', name='VisualModel', color=[0.2, 0.8, 1.0, 1.0], lineWidth=5)

    # --- 血管环境 ---
    vessel_path = '/home/wen-zheng/meshes/vessel.stl'
    if os.path.exists(vessel_path):
        vessel = rootNode.addChild('Vessel')
        vessel.addObject('MeshSTLLoader', name='loader', filename=vessel_path)
        vessel.addObject('MeshTopology', src='@loader')
        vessel.addObject('MechanicalObject', showObject=False)
        vessel.addObject('TriangleCollisionModel', simulated=False)
        v_vis = vessel.addChild('Visu')
        v_vis.addObject('OglModel', src='@../loader', color=[1, 0, 0, 0.3])

    return rootNode
