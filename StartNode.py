from lcmraft.states.neutral import Neutral
from lcmraft.servers.server import ServerDeamon
import sys
import lcm

name = sys.argv[1]

_lcm = lcm.LCM()

state = Neutral()
node = ServerDeamon(name,state,[],_lcm)
node.daemon = True

_lcm_heartbeat_handler   = lambda channel,data:node.HandleHeartBeat(channel,data)
_lcm_appendentry_handler = lambda channel,data:node.HandleAppendEntries(channel,data)
_lcm_requestvote_handler = lambda channel,data:node.HandleRequestVote(channel,data)
_lcm_response_handler    = lambda channel,data:node.HandleResponse(channel,data)
_lcm_voteresponse_handler= lambda channel,data:node.HandleVoteResponse(channel,data)
_lcm_membership_handler  = lambda channel,data:node.HandleMemberShip(channel,data)
_lcm_clientstatus_handler= lambda channel,data:node.HandleClientStatus(channel,data)


_lcm.subscribe("HEARTBEAT",_lcm_heartbeat_handler)
_lcm.subscribe(name + "_APPEND_ENTRIES",_lcm_appendentry_handler)
_lcm.subscribe(name + "_REQUEST_VOTE",_lcm_requestvote_handler)
_lcm.subscribe(name + "_VOTE_RESPONSE",_lcm_voteresponse_handler)
_lcm.subscribe(name + "_RESPONSE",_lcm_response_handler)
_lcm.subscribe(name + "_MEMBERSHIP",_lcm_membership_handler)
_lcm.subscribe(name + "_CLIENT_STATUS",_lcm_clientstatus_handler)


node.start()

try:
    while True:
        _lcm.handle()
except KeyboardInterrupt:
    print "Exiting lcm thread"

