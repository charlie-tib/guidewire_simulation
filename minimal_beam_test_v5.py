import Sofa
import Sofa.Core
import SofaRuntime

def createScene(rootNode):
    SofaRuntime.importPlugin('BeamAdapter')
    instrument = rootNode.addChild('instrument')
    obj = instrument.addObject('WireRestShape', name='RestShape', template='Rigid3d')
    print('Available DataFields:', [d.getName() for d in obj.getDataFields()])
    obj.length.value = 0.5
    obj.straightLength.value = 0.5
    obj.numEdges.value = 30
    print('Set values successfully')
    return rootNode
