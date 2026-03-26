import Sofa
import Sofa.Core
import SofaRuntime

def createScene(rootNode):
    SofaRuntime.importPlugin('BeamAdapter')
    instrument = rootNode.addChild('instrument')
    obj = instrument.addObject('WireRestShape', name='RestShape', template='Rigid3d',
                         straightLength=0.5, length=0.5, numEdges=30)
    print('Success with attributes')
    return rootNode
