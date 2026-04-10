import Sofa
import Sofa.Core
import SofaRuntime

def createScene(rootNode):
    SofaRuntime.importPlugin('BeamAdapter')
    instrument = rootNode.addChild('instrument')
    # Try adding without attributes first to see what attributes it HAS
    obj = instrument.addObject('WireRestShape', name='RestShape', template='Rigid3d')
    print('Attributes:', list(obj.__dict__.keys()))
    return rootNode
