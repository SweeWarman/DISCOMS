import sys, time, lcm, csv
sys.path.append("./RAFTLiTE")

from RAFTLiTE.states.neutral import Neutral
from RAFTLiTE.servers.server import ServerDeamon
from RAFTLiTE.Communication.Comm import MsgBoard
from RAFTLiTE.Communication.LcmServer import LcmServer

from Agent import UAVAgent
from Animation import AgentAnimation

# Obtain command line arguments
id  = float(sys.argv[1])
x   = float(sys.argv[2])
y   = float(sys.argv[3])
z   = float(sys.argv[4])
vx  = float(sys.argv[5])
vy  = float(sys.argv[6])
vz  = float(sys.argv[7])
log = sys.argv[8]


print "Launching agent"

# Instantiate an agent
UAV = UAVAgent(id,[x,y,z],[vx,vy,vz])

# Add an intersection for the agent
UAV.AddIntersections(0,0,0,0)
UAV.daemon = True

# Instantiate RAFT server
board = MsgBoard()
state = Neutral()
node = ServerDeamon("NODE"+str(id),state,[],board)
node.daemon = True

lcm = LcmServer(node._server,board)
lcm.daemon = True

# Provide the lcm hander to the UAV to publish relevant topics
UAV.SetLcmHandle(lcm._lcm)

# Provide the RAFT server to the UAV to send updates to the RAFT leader
UAV.SetServer(node._server)

# Subscribe to the relevant LCM channels
if log == "True":
    lcm._lcm.subscribe("POSITION",UAV.HandleTrafficPosition)


# Start the LCM server
lcm.start()

# Start the RAFT server thread
node.start()

# Launch the UAV
UAV.start()

# Run the UAV only for a specified duration

duration = 45
t1 = time.time()
t2 = time.time()
try:
    while abs(t2 - t1) < duration:
        t2 = time.time()
        lcm._lcm.handle_timeout(100)
    print "Exiting lcm wait loop"
except KeyboardInterrupt:
    print "Exiting UAV thread"

# Terminate the UAV and RAFT server threads
UAV.stop()
node.stop()
UAV.join()
node.join()
print "Finished running program"

logfile = id + '.csv'
with open(logfile, 'wb') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=' ',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
    for element in UAV.blacbox:
        spamwriter.writerow(element)

# Visualize results
"""
Visualation of agents
"""
if log == "True":
    print "Starting animation"
    anim = AgentAnimation(-200,-200,200,200)
    anim.AddAgent("vehicle1",2.5,'r')
    anim.AddAgent("vehicle2",2.5,'b')
    #anim.AddAgent("vehicle3",2.5,'g')

    anim.AddData(UAV.trafficTraj)
    anim.run()
