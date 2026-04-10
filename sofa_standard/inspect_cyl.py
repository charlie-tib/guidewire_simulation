import sys
import os
sys.path.insert(0, '/home/wen-zheng/miniconda3/envs/sofanew/lib/python3.12/site-packages')
import Sofa.Core
root = Sofa.Core.Node("root")
cyl = root.addObject("CylinderVisualModel")
print(cyl.getDataContext())
print([k for k in dir(cyl) if not k.startswith('_')])
