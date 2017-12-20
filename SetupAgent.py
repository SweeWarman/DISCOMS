from Agent import UAVAgent
import lcm
import sys
import time
from msg import acState_t
from msg import jobprop_t
from msg import xtimerequest_t
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def HandleTrafficPosition(channel,data):
    global UAV
    msg = acState_t.decode(data)
    if msg.aircraftID != UAV.id:
        UAV.UpdateTrafficState(msg)

def HandleJobProperties(channel,data):
    global UAV
    msg = jobprop_t.decode(data)
    if msg.aircraftID != UAV.id:
        UAV.UpdateIntruderCrossingTime(msg)

def HandleRequest(channel,data):
    global UAV
    msg = xtimerequest_t.decode(data)
    if msg.toAircraftID == UAV.id:
        UAV.ProcessRequest(msg)


from PolynomialTime import PolynomialTime


id = int(sys.argv[1])
x = float(sys.argv[2])
y = float(sys.argv[3])
z = float(sys.argv[4])
vx = float(sys.argv[5])
vy = float(sys.argv[6])
vz = float(sys.argv[7])

lc = lcm.LCM()
subscription = lc.subscribe("POSITION",HandleTrafficPosition)
subscription = lc.subscribe("JOB",HandleJobProperties)
subscription = lc.subscribe("REQUEST",HandleRequest)

if sys.argv[8] == "True":
    log = True
else:
    log = False

UAV = UAVAgent(id,x,y,z,vx,vy,vz,lc,log)
UAV.AddIntersections(0,0,100,0)
UAV.start()

t1 = time.time()
t2 = 0

try:
    while t2 - t1 < 20:
        t2 = time.time()
        lc.handle()
    print "Exiting lcm wait loop"
except KeyboardInterrupt:
    print "Exiting UAV thread"
UAV.status = False
UAV.join()
print "Finished running program"

"""
Code for visualization
"""
fig = plt.figure()
ax  = plt.axes(xlim=(-200,200),ylim=(-200,200))
agt1 = plt.Circle((0.0,0.0),radius = 2.5,fc='b')
agt2 = plt.Circle((10.0,25.0),radius = 2.5,fc='r')
ax.add_patch(agt1)
ax.add_patch(agt2)
ownship = agt1
agt = [agt2]

def init():
    return agt

def animate(i):
    if i < len(UAV.trajx):
        traj = [UAV.trajx[i], UAV.trajy[i]]
        ownship.center = (traj[0],traj[1])
        for j,vehicle in enumerate(agt):
            id = j + 1
            vehicle.center = (UAV.trafficTraj[id][i][0],UAV.trafficTraj[id][i][1])
    return agt

if log:
    anim = animation.FuncAnimation(fig, animate,
                                   init_func=init,
                                   frames=1000,
                                   interval=150,
                                   blit=False)


    plt.show()


