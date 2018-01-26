import sys, time, lcm
from Agent import UAVAgent
from lcmraft.states.neutral import Neutral
from lcmraft.servers.server import ServerDeamon
from Animation import AgentAnimation

# Obtain command line arguments
id  = sys.argv[1]
x   = float(sys.argv[2])
y   = float(sys.argv[3])
z   = float(sys.argv[4])
vx  = float(sys.argv[5])
vy  = float(sys.argv[6])
vz  = float(sys.argv[7])
log = sys.argv[8]

# Create lcm handle to publish/subscribe channels
_lcm = lcm.LCM()

# Instantiate an agent
UAV = UAVAgent(id,[x,y,z],[vx,vy,vz])

# Add an intersection for the agent
UAV.AddIntersections(0,0,0,0)
UAV.daemon = True

# Instantiate RAFT server
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

if log is "True":
    _lcm.subscribe("POSITION",UAV.HandleTrafficPosition)

_lcm.subscribe("HEARTBEAT",node.HandleHeartBeat)
_lcm.subscribe("CLIENT_STATUS",node.HandleClientStatus)
_lcm.subscribe(id + "_APPEND_ENTRIES",node.HandleAppendEntries)
_lcm.subscribe(id + "_REQUEST_VOTE",node.HandleRequestVote)
_lcm.subscribe(id + "_VOTE_RESPONSE",node.HandleVoteResponse)
_lcm.subscribe(id + "_RESPONSE",node.HandleResponse)
_lcm.subscribe(id + "_MEMBERSHIP",node.HandleMemberShip)


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
Visualation of agents
"""
if log is "True":
    anim = AgentAnimation(-200,-200,200,200)
    anim.AddAgent(2.5,'r')
    anim.AddAgent(2.5,'b')

    anim.AddData(UAV.trafficTraj)
    anim.run()
