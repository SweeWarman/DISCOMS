
from Animation import *

Data = {}
Data["vehicle1"] = []
for i in range(100000):
    Data["vehicle1"].append((-100.0 + i,0,0))

anim = AgentAnimation(-200,-200,200,200)
anim.AddAgent(2.5,'r')
anim.AddData(Data)
anim.run()