pyrun("x=1")
pyrun("print(x)")
pyrun("x=x+1")
pyrun("print(x)")

pyrun("import pyrvo2")
pyrun("import numpy as np")
pyrun("import math")
pyrun("from pyrvo2 import *")

%m_sim = pyrun("sim=RVOSimulator()","sim");
pyrun("sim=RVOSimulator()")
pyrun("sim.setTimeStep(0.25)")


