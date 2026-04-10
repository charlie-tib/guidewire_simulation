import Sofa
import Sofa.Core

def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', pluginName=['BeamAdapter', 'Sofa.Component.Topology.Container.Dynamic'])
    instrument = rootNode.addChild('instrument')
    instrument.addObject('EdgeSetTopologyContainer', name='topo')
    instrument.addObject('WireRestShape', name='RestShape', template='Rigid3d',
                         straightLength=0.5, length=0.5, numEdges=30)
    print('WireRestShape added successfully')
    return rootNode
