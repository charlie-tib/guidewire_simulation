# SOFA 仿真引擎配置，包括求解器、物理时间步长及插件加载。
import Sofa


class Simulator(Sofa.Core.Controller):
    '''
    A class used to define the physics and the solver of the SOFA simulation.

    :param root_node: The sofa root node
    :type root_node:
    :param dt: The time step (s)
    :type dt: float
    :param gravity: The gravity verctor (m/s^2)
    :type gravity: float
    :param friction_coef: The coeficient of friction
    :type friction_coef: float
    '''

    def __init__(
            self,
            root_node,
            dt=0.001,
            gravity=[0, 0, 0],
            friction_coef=.04,
            *args, **kwargs):

        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.root_node = root_node
        self.dt = dt
        self.gravity = gravity
        self.friction_coef = friction_coef

        # v25.12+ 插件加载
        plugins = [
            "Sofa.Component.AnimationLoop",
            "Sofa.Component.Collision.Detection.Algorithm",
            "Sofa.Component.Collision.Detection.Intersection",
            "Sofa.Component.Collision.Geometry",
            "Sofa.Component.Collision.Response.Contact",
            "Sofa.Component.Constraint.Lagrangian.Correction",
            "Sofa.Component.Constraint.Lagrangian.Solver", # 用于 LCP
            "Sofa.Component.Constraint.Projective",
            "Sofa.Component.IO.Mesh",
            "Sofa.Component.LinearSolver.Direct",
            "Sofa.Component.Mapping.Linear",
            "Sofa.Component.Mass",
            "Sofa.Component.MechanicalLoad",
            "Sofa.Component.ODESolver.Backward",
            "Sofa.Component.SolidMechanics.Spring",
            "Sofa.Component.StateContainer",
            "Sofa.Component.Setting",
            "Sofa.Component.Topology.Container.Constant",
            "Sofa.Component.Topology.Container.Dynamic",
            "Sofa.Component.Topology.Container.Grid",
            "Sofa.Component.Topology.Mapping",
            "Sofa.Component.Visual",
            "Sofa.GL.Component.Rendering3D",
            "SofaPython3",
            "BeamAdapter",
            "SoftRobots"
        ]
        for p in plugins:
            self.root_node.addObject('RequiredPlugin', name=f"Import{p}", pluginName=p)

        self.root_node.dt = self.dt
        self.root_node.animate = True
        self.root_node.gravity = self.gravity

        self.root_node.addObject(
            'VisualStyle',
            displayFlags='showVisualModels hideBehaviorModels \
                hideCollisionModels hideMappings hideForceFields \
                    hideInteractionForceFields')
        self.root_node.addObject(
            'FreeMotionAnimationLoop')
        self.lcp_solver = self.root_node.addObject(
            'LCPConstraintSolver',
            mu=str(friction_coef),
            tolerance='1e-6',
            maxIt='10000',
            build_lcp='true')
        self.root_node.addObject(
            'CollisionPipeline',
            draw='0',
            depth='6',
            verbose='0')
        self.root_node.addObject(
            'BruteForceDetection',
            name='N2')
        self.root_node.addObject(
            'LocalMinDistance',
            contactDistance='0.0001',
            alarmDistance='0.0005',
            name='localmindistance',
            angleCone='0.02')
        self.root_node.addObject(
            'CollisionResponse',
            name='Response',
            response='FrictionContactConstraint')
        # CollisionGroup is deprecated or removed in many v25+ setups, removing it.

        # set backbround color
        self.root_node.addObject('BackgroundSetting', color='1 1 1')
