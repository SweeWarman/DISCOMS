from Agent import UAVAgent
import lcm
import sys
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from lcmraft.states.neutral import Neutral
from lcmraft.servers.server import ServerDeamon

id  = sys.argv[1]
x   = float(sys.argv[2])
y   = float(sys.argv[3])
z   = float(sys.argv[4])
vx  = float(sys.argv[5])
vy  = float(sys.argv[6])
vz  = float(sys.argv[7])
log = sys.argv[8]

_lcm = lcm.LCM()

UAV = UAVAgent(id,[x,y,z],[vx,vy,vz])
UAV.AddIntersections(0,0,0,0)
UAV.daemon = True

state = Neutral()
node = ServerDeamon(id,state,[],_lcm)
node.daemon = True

UAV.SetLcmHandle(_lcm)
UAV.SetServer(node._server)

_lcm_traffic_handler     = lambda channel,data:UAV.HandleTrafficPosition(channel,data)
_lcm_heartbeat_handler   = lambda channel,data:node.HandleHeartBeat(channel,data)
_lcm_appendentry_handler = lambda channel,data:node.HandleAppendEntries(channel,data)
_lcm_requestvote_handler = lambda channel,data:node.HandleRequestVote(channel,data)
_lcm_response_handler    = lambda channel,data:node.HandleResponse(channel,data)
_lcm_voteresponse_handler= lambda channel,data:node.HandleVoteResponse(channel,data)
_lcm_membership_handler  = lambda channel,data:node.HandleMemberShip(channel,data)
_lcm_clientstatus_handler= lambda channel,data:node.HandleClientStatus(channel,data)

_lcm.subscribe("POSITION",_lcm_traffic_handler)
_lcm.subscribe("HEARTBEAT",_lcm_heartbeat_handler)
_lcm.subscribe("CLIENT_STATUS",_lcm_clientstatus_handler)
_lcm.subscribe(id + "_APPEND_ENTRIES",_lcm_appendentry_handler)
_lcm.subscribe(id + "_REQUEST_VOTE",_lcm_requestvote_handler)
_lcm.subscribe(id + "_VOTE_RESPONSE",_lcm_voteresponse_handler)
_lcm.subscribe(id + "_RESPONSE",_lcm_response_handler)
_lcm.subscribe(id + "_MEMBERSHIP",_lcm_membership_handler)


node.start()
UAV.start()

t1 = time.time()
t2 = 0

try:
    while abs(t2 - t1) > 30:
        t2 = time.time()
        _lcm.handle()
    print "Exiting lcm wait loop"
except KeyboardInterrupt:
    print "Exiting UAV thread"

UAV.stop()
node.stop()
UAV.join()
node.join()
print "Finished running program"

"""
Code for visualization
"""

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
"""