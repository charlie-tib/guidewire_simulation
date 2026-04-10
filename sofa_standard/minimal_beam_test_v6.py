import Sofa
import Sofa.Core
import SofaRuntime

def createScene(rootNode):
    SofaRuntime.importPlugin('BeamAdapter')
    SofaRuntime.importPlugin('Sofa.Component.Topology.Container.Dynamic')
    instrument = rootNode.addChild('instrument')
    instrument.addObject('RodStraightSection', name='Section', 
                         length=0.5, radius=0.000665, 
                         nbEdgesCollis=30, nbEdgesVisu=30, 
                         youngModulus=170e6)
    instrument.addObject('WireRestShape', name='RestShape', template='Rigid3d',
                         wireMaterials='@Section')
    print('WireRestShape with wireMaterials added successfully')
    return rootNode
