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

# Provide the lcm hander to the UAV to publish relevant topics
UAV.SetLcmHandle(_lcm)

# Provide the RAFT server to the UAV to send updates to the RAFT leader
UAV.SetServer(node._server)

# Subscribe to the relevant LCM channels
if log == "True":
    _lcm.subscribe("POSITION",UAV.HandleTrafficPosition)

_lcm.subscribe("HEARTBEAT",node.HandleHeartBeat)
_lcm.subscribe("CLIENT_STATUS",node.HandleClientStatus)
_lcm.subscribe(id + "_APPEND_ENTRIES",node.HandleAppendEntries)
_lcm.subscribe(id + "_REQUEST_VOTE",node.HandleRequestVote)
_lcm.subscribe(id + "_VOTE_RESPONSE",node.HandleVoteResponse)
_lcm.subscribe(id + "_RESPONSE",node.HandleResponse)
_lcm.subscribe(id + "_MEMBERSHIP",node.HandleMemberShip)

# Start the RAFT server thread
node.start()

# Launch the UAV
UAV.start()

# Run the UAV only for a specified duration

duration = 30
t1 = time.time()
t2 = time.time()
try:
    while abs(t2 - t1) < duration:
        t2 = time.time()
        _lcm.handle()
    print "Exiting lcm wait loop"
except KeyboardInterrupt:
    print "Exiting UAV thread"

# Terminate the UAV and RAFT server threads
UAV.stop()
node.stop()
UAV.join()
node.join()
print "Finished running program"

# Visualize results
"""
Visualation of agents
"""
if log == "True":
    print "Starting animation"
    anim = AgentAnimation(-200,-200,200,200)
    anim.AddAgent(2.5,'r')
    anim.AddAgent(2.5,'b')

    anim.AddData(UAV.trafficTraj)
    anim.run()
