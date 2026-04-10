import Sofa
import numpy as np


class Instrument(Sofa.Core.Controller):
    '''
    A class used to define instrument objects and to build the SOFA mechanical,
    collision and visual models of the magnetic instrument.
    '''

    def __init__(
            self,
            root_node,
            magnets,
            name='mag_instrument',
            length_body=0.5,
            length_tip=0.0034,
            outer_diam=0.00133,
            inner_diam=0.0008,
            young_modulus_body=170e6,
            young_modulus_tip=21e6,
            num_elem_body=30,
            num_elem_tip=3,
            nume_nodes_viz=600,
            T_start_sim=[0., 0., 0., 0., 0., 0., 1.],
            fixed_directions=[0, 0, 0, 0, 0, 0],
            color=[0.2, .8, 1., 1.],
            segments=None,
            main_direction=[0, 0, 1],
            use_navigation=True,
            *args, **kwargs):

        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.root_node = root_node
        self.main_direction = main_direction

        self.magnets = magnets
        # 索引计算逻辑保持不变，确保控制器能找到 3 个磁铁
        self.index_mag = np.nonzero(self.magnets)[0]
        self.outer_diam = outer_diam
        self.inner_diam = inner_diam

        # 如果提供了 segments，计算总长度和总单元数
        if segments is not None:
            total_len = sum(seg['length'] for seg in segments)
            total_elements = sum(seg['num_elem'] for seg in segments)
            self.num_elem_body = total_elements - 1
            self.num_elem_tip = 1
            length_body = total_len
            length_tip = 0.0
        else:
            self.num_elem_body = num_elem_body
            self.num_elem_tip = num_elem_tip
            total_len = length_body + length_tip
            total_elements = num_elem_body + num_elem_tip

        self.nume_nodes_viz = nume_nodes_viz
        self.insertion_len = 0.
        self.color = color
        self.outer_diam_qu = ((outer_diam/2.)**4.-(inner_diam/2.)**4.)**(1./4.)
        self.fixed_directions = fixed_directions

        topoLines_guide = self.root_node.addChild(name+'_topo_lines')

        # [STABLE DVS] Merge adjacent segments with identical Young's Modulus to reduce section boundaries
        merged_segments = []
        if segments is not None:
            for seg in segments:
                if not merged_segments or merged_segments[-1]['E'] != seg['E']:
                    merged_segments.append(seg.copy())
                else:
                    merged_segments[-1]['length'] += seg['length']
                    merged_segments[-1]['num_elem'] += seg['num_elem']
        
        section_paths = []
        if segments is not None:
            for i, seg in enumerate(merged_segments):
                sec_name = f'Section_{i}'
                topoLines_guide.addObject('RodStraightSection', 
                                           name=sec_name,
                                           youngModulus=seg['E'],
                                           poissonRatio=0.33,
                                           radius=self.outer_diam_qu/2.0,
                                           length=seg['length'],
                                           nbEdgesCollis=seg['num_elem'],
                                           nbEdgesVisu=seg['num_elem'])
                section_paths.append(f'@{sec_name}')
        else:
            # Fallback to simple body/tip if no segments provided
            topoLines_guide.addObject('RodStraightSection', name='BodySection',
                                       youngModulus=young_modulus_body, poissonRatio=0.33,
                                       radius=self.outer_diam_qu/2.0, length=length_body,
                                       nbEdgesCollis=num_elem_body, nbEdgesVisu=num_elem_body)
            topoLines_guide.addObject('RodStraightSection', name='TipSection',
                                       youngModulus=young_modulus_tip, poissonRatio=0.33,
                                       radius=self.outer_diam_qu/2.0, length=length_tip,
                                       nbEdgesCollis=num_elem_tip, nbEdgesVisu=num_elem_tip)
            section_paths = ['@BodySection', '@TipSection']

        topoLines_guide.addObject(
            'WireRestShape',
            name='InstrRestShape',
            template='Rigid3d',
            wireMaterials=" ".join(section_paths),
            printLog=True)

        topoLines_guide.addObject(
            'EdgeSetTopologyContainer',
            name='meshLinesGuide')
        topoLines_guide.addObject(
            'EdgeSetTopologyModifier',
            name='Modifier')
        topoLines_guide.addObject(
            'EdgeSetGeometryAlgorithms',
            name='GeomAlgo',
            template='Rigid3d')
        topoLines_guide.addObject(
            'MechanicalObject',
            name='dofTopo2',
            template='Rigid3d',
            showObject=False) # 正常运行时关闭调试坐标轴

        self.InstrumentCombined = self.root_node.addChild(name)
        self.InstrumentCombined.addObject(
            'EulerImplicitSolver',
            rayleighStiffness=0.5, # Increased damping to suppress jumping
            printLog=False,
            rayleighMass=0.1)     # Mass damping also increased
        self.InstrumentCombined.addObject(
            'BTDLinearSolver',
            verification=False,
            subpartSolve=False, verbose=False)
        self.RG = self.InstrumentCombined.addObject(
            'RegularGridTopology',
            name='meshLinesCombined',
            zmax=1, zmin=1,
            nx=self.num_elem_body+self.num_elem_tip + 1, ny=1, nz=1,
            xmax=length_body+length_tip, xmin=0, ymin=0, ymax=0)
        self.MO = self.InstrumentCombined.addObject(
            'MechanicalObject',
            showIndices=False,
            name='DOFs',
            template='Rigid3d',
            showObject=False)

        # [STABLE STATIC] Handle non-navigation mode
        self.use_navigation = use_navigation
        self.main_direction = main_direction
        
        self.MO.init()
        indicesAll = list(range(self.num_elem_body + self.num_elem_tip + 1))

        # [REVERTED] Manual rotation caused SEGFAULT. Using Frozen IRC instead.
        self.use_navigation = use_navigation
        self.main_direction = main_direction
        
        forcesList = ""
        for i in range(0, self.num_elem_body+self.num_elem_tip):
            forcesList += " 0 0 0 0 0 0 "

        indicesList = list(range(0, self.num_elem_body+self.num_elem_tip))

        self.IC = self.InstrumentCombined.addObject(
            'WireBeamInterpolation',
            WireRestShape='@../'+name+'_topo_lines'+'/InstrRestShape',
            printLog=True,
            name='InterpolGuide')
        self.InstrumentCombined.addObject(
            'AdaptiveBeamForceFieldAndMass',
            massDensity=155.0,
            name='GuideForceField',
            interpolation='@InterpolGuide')

        self.CFF = self.InstrumentCombined.addObject(
            'ConstantForceField',
            name='ConstantForceField',
            indices=indicesList,
            forces=forcesList,
            indexFromEnd=True)

        self.CFF_visu = self.InstrumentCombined.addObject(
            'ConstantForceField',
            name='ConstantForceFieldViz',
            indices=0,
            forces='0 0 0 0 0 0',
            showArrowSize=1.e2)

        self.IRC = self.InstrumentCombined.addObject(
            'InterventionalRadiologyController',
            xtip=[0.001 if self.use_navigation else 0.110], name='m_ircontroller',
            instruments='InterpolGuide',
            step=0.0005,
            printLog=True,
            # [STATIC FIX] Disable listening and speed if in static mode
            listening=self.use_navigation,
            template='Rigid3d',
            startingPos=T_start_sim,
            rotationInstrument=[0.],
            speed=1e-12 if self.use_navigation else 0.0,
            mainDirection=self.main_direction,
            threshold=5e-9,
            controlledInstrument=0)
        
        self.InstrumentCombined.addObject(
            'LinearSolverConstraintCorrection',
            wire_optimization='true',
            printLog=False)

        self.InstrumentCombined.addObject(
            'FixedProjectiveConstraint',
            indices=0,
            name='FixedConstraint')
        # [REMOVED] RestShapeSpringsForceField often causes "Ghost Stretching" when fighting with IRC

        # restrict DOF of nodes
        self.InstrumentCombined.addObject(
            'PartialFixedProjectiveConstraint', # Optimized for v25.06 stability
            indices=indicesAll,
            fixedDirections=self.fixed_directions,
            fixAll=True)

        # Collision model
        Collis = self.InstrumentCombined.addChild(name+'_collis')
        Collis.addObject('EdgeSetTopologyContainer', name='collisEdgeSet')
        Collis.addObject('EdgeSetTopologyModifier', name='colliseEdgeModifier')
        Collis.addObject('MechanicalObject', name='CollisionDOFs')
        Collis.addObject(
            'MultiAdaptiveBeamMapping',
            controller='../m_ircontroller',
            useCurvAbs=True, printLog=False,
            name='collisMap')
        Collis.addObject('LineCollisionModel', proximity=0.0002, group=1)
        Collis.addObject('PointCollisionModel', proximity=0.0002, group=1)

        # visualization sofa (Main Tube)
        CathVisu = self.InstrumentCombined.addChild(name+'_viz')
        CathVisu.addObject('MechanicalObject', name='QuadsCatheter')
        CathVisu.addObject('QuadSetTopologyContainer', name='ContainerCath')
        CathVisu.addObject('QuadSetTopologyModifier', name='Modifier')
        CathVisu.addObject('QuadSetGeometryAlgorithms', name='GeomAlgo', template='Vec3d')
        CathVisu.addObject(
            'Edge2QuadTopologicalMapping',
            flipNormals='true',
            input='@../../'+name+'_topo_lines'+'/meshLinesGuide',
            nbPointsOnEachCircle='10',
            output='@ContainerCath',
            radius=self.outer_diam_qu/2,
            tags='catheter')
        CathVisu.addObject(
            'AdaptiveBeamMapping',
            interpolation='@../InterpolGuide',
            input='@../DOFs',
            isMechanical='false',
            name='VisuMapCath',
            output='@QuadsCatheter',
            printLog='1',
            useCurvAbs='1')

        VisuOgl = CathVisu.addChild('VisuOgl')
        VisuOgl.addObject(
            'OglModel',
            quads='@../ContainerCath.quads',
            color=self.color,
            material='texture Ambient 1 0.2 0.2 0.2 0.0 Diffuse 1 1.0 1.0 1.0 1.0 Specular 1 1.0 1.0 1.0 1.0 Emissive 0 0.15 0.05 0.05 0.0 Shininess 1 20',
            name='VisualCatheter')
        VisuOgl.addObject(
            'IdentityMapping',
            input='@../QuadsCatheter',
            output='@VisualCatheter',
            name='VisuCathIM')
