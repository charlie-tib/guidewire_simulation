import Sofa
import numpy as np
import os
import sys

# Ensure path is correct
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(current_dir, 'mCR_simulator-master/python'))

from mcr_sim_prb import mcr_instrument, mcr_external_magnet, mcr_simulator, mcr_magnet, mcr_mag_controller

def createScene(root_node):
    # 用极软和极硬的对比来验证变刚度是否生效
    segments = [
        {'length': 0.050, 'E': 1e9, 'num_elem': 50, 'is_magnet': False}, # 硬
        {'length': 0.030, 'E': 1e6, 'num_elem': 30, 'is_magnet': False}, # 极软 (软铰链)
        {'length': 0.010, 'E': 1e9, 'num_elem': 10, 'is_magnet': True, 'polarity': 1.0} # 硬磁铁
    ]
    
    magnets_list = [0.0] * 91 # 50+30+10+1
    magnets_list[0] = mcr_magnet.Magnet(length=0.01, outer_diam=0.002, inner_diam=0, remanence=1.2, polarity=1.0)
    
    simulator = mcr_simulator.Simulator(root_node=root_node, gravity=[0, 0, 0])
    
    # 放置 EPM 在侧向，观察是否只在“极软”处弯折
    external_magnet = mcr_external_magnet.ExternalMagnet(
        root_node=root_node, name='EPM', init_pos=[0.05, 0, 0.08], init_rot=[0, 90, 0])
    
    instrument = mcr_instrument.Instrument(
        name='StiffnessTest',
        root_node=root_node,
        magnets=magnets_list,
        segments=segments,
        T_start_sim=[0, 0, 0, 0, 0, 0, 1],
        use_navigation=False
    )
    
    class TipPositionPrinter(Sofa.Core.Controller):
        def __init__(self, instrument, *args, **kwargs):
            Sofa.Core.Controller.__init__(self, *args, **kwargs)
            self.instrument = instrument
            self.step = 0
        def onAnimateBeginEvent(self, event):
            self.step += 1
            if self.step == 500:
                pos = self.instrument.MO.position.value[-1]
                print(f"STIFFNESS_TEST_TIP_POS: {pos[0]} {pos[1]} {pos[2]}")
                # Exit
                sys.exit(0)

    root_node.addObject(TipPositionPrinter(instrument))
    return root_node
