import sys
import lcm
from lcmraft.LcmRaftMessages import client_status_t

# name of server to send message to
name = sys.argv[1]
vehicleID = sys.argv[2]

msg = client_status_t()
msg.intersectionID = 1
msg.vehicleID = vehicleID
msg.entryTime = 123
msg.exitTime  = 230
msg.crossingTime = 180


lc = lcm.LCM()
lc.publish(name+"_CLIENT_STATUS",msg.encode())




try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    print "Exiting lcm thread"


