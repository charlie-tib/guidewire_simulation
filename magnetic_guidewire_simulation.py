import Sofa
import Sofa.Core
import SofaRuntime

def createScene(rootNode):
    # ==========================================
    # 1. 插件与环境配置 (必须首先加载)
    # ==========================================
    rootNode.addObject('RequiredPlugin', pluginName=[
        'Sofa.Component.LinearSolver.Direct',
        'Sofa.Component.ODESolver.Backward',
        'Sofa.Component.StateContainer',
        'Sofa.Component.Topology.Container.Dynamic',
        'Sofa.Component.Constraint.Projective',
        'Sofa.Component.Visual',
        'Sofa.Component.Mapping.Linear',
        'BeamAdapter',
        'SofaPython3'
    ])

    rootNode.dt = 0.01
    rootNode.gravity = [0, -9810, 0] # 注意单位量级 (mm/s^2)
    rootNode.addObject('DefaultAnimationLoop')
    
    # 强制开启显示：显示行为模型、节点和视觉
    rootNode.addObject('VisualStyle', displayFlags="showBehaviorModels showCollisionModels showVisual showWireframe")

    # ==========================================
    # 2. 导丝机构建模 (Mechanism Node)
    # ==========================================
    num_nodes = 20
    wire_length = 100.0
    
    guidewire = rootNode.addChild('Guidewire')

    # 2.1 高级求解器 (Stiff System Solver)
    guidewire.addObject('EulerImplicitSolver', name='Solver', rayleighStiffness=0.1, rayleighMass=0.1)
    guidewire.addObject('BTDLinearSolver', name='LinearSolver')

    # 2.2 显式拓扑连接 (解决 Edge_List 报错的核心)
    indices = [[i, i+1] for i in range(num_nodes-1)]
    guidewire.addObject('EdgeSetTopologyContainer', name='Topology', edges=indices)
    guidewire.addObject('EdgeSetTopologyModifier')

    # 2.3 状态空间定义 (Rigid3d: 位置+旋转)
    pos_init = [[i * (wire_length/(num_nodes-1)), 0, 0, 0, 0, 0, 1] for i in range(num_nodes)]
    dofs = guidewire.addObject('MechanicalObject', name='DOFs', template='Rigid3d', position=pos_init)

    # 2.4 材料属性重构 (针对 v25.12 属性报错的修复)
    # 技巧：我们先创建对象，不传参数，避开构造函数属性检查
    section = guidewire.addObject('RodStraightSection', name='Material')
    # 使用 setData 手动注入，这是最稳的方法
    section.setData('youngModulus', 1e7)
    section.setData('radius', 0.4)
    # 如果系统依然报错找不到 shearModulus，我们就不设它，SOFA 会根据杨氏模量自动推算
    try:
        section.setData('shearModulus', 5e6)
    except:
        print("[Warning] shearModulus set failed, using default Poisson ratio calculation.")

    # 2.5 建立力学握手 (WireRestShape & Interpolation)
    # 显式链接：路径必须精确
    guidewire.addObject('WireRestShape', name='RestShape', 
                        template='Rigid3d', 
                        wireMaterials='@Material')
    
    guidewire.addObject('WireBeamInterpolation', name='Interpolation', 
                        WireRestShape='@RestShape')
    
    guidewire.addObject('AdaptiveBeamForceFieldAndMass', 
                        name='BeamForceField',
                        interpolation='@Interpolation', 
                        massDensity=0.0001)

    # 2.6 边界条件 (修复 FixedConstraint 更名问题)
    # v25.12 建议使用 FixedProjectiveConstraint
    guidewire.addObject('FixedProjectiveConstraint', name='FixBase', indices=0)

    # ==========================================
    # 3. 视觉模型 (可视化修复：确保能看见连线)
    # ==========================================
    visual = guidewire.addChild('VisualModel')
    visual.addObject('MechanicalObject', template='Vec3d', name='VisualDOFs')
    # 核心：将力学层的 Rigid3d (带旋转) 映射到视觉层的 Vec3d (纯坐标)
    visual.addObject('BeamLinearMapping', isMechanical=True, input='@../DOFs', output='@VisualDOFs')
    
    # 添加一个红色的显示点和黄色连线
    visual.addObject('SphereCollisionModel', radius=0.6, color=[1, 0, 0, 1])
    visual.addObject('LineCollisionModel', color=[1, 1, 0, 1])

    print("Expert: 导丝硬件结构重构完成，已绕开 v25.12 属性冲突。")
    return rootNode