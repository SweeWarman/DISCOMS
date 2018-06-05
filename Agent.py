import numpy as np
import threading
from exlcm import acState_t,jobprop_t
from PolynomialTime import PolynomialTime
from RAFTLiTE.states.leader import Leader
from RAFTLiTE.states.follower import Follower
from RAFTLiTE.states.state import EntryType
from RAFTLiTE.Communication.LcmRaftMessages import client_status_t,request_membership_t
from Vehicle import Vehicle
import time

class UAVAgent(threading.Thread):
    def __init__(self,id,position,velocity):
        threading.Thread.__init__(self)
        self.threadLock = threading.Lock()
        self.ownship = Vehicle(id,position,velocity)
        self.trafficTraj = {}
        self.lcm = None
        self.log = None
        self.pt0 = time.time()
        self.bt0 = time.time()
        self.nt0 = time.time()
        self.status = True
        self.intersections = {}
        self.crossingTimes = {}
        self.xtrackdev = 40
        self.schedules = {}
        self.newSchedule = False
        self.delta = 10
        self.trajectory = []
        self.prevCrossingT = 0
        self.speed = np.sqrt(self.ownship.vx**2 + self.ownship.vy**2 + self.ownship.vz**2)
        self.zone = 50
        self.server = None
        self.lastLogLength = 0
        self.shutdownSent = False
        self._stop_event = threading.Event()
        self.lastProcessedIndex = 0
        self.computeSent = False
        self.offsetSign = 1
        self.blacbox = []
        self._atLeastOneMsgSent = False
        #TODO: analyze relation between vmin,vmax,xtracdev

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def SetServer(self,svr):
        self.server = svr

    def SetLcmHandle(self,lcm):
        self.lcm = lcm

    def AddIntersections(self,id,x,y,z):
        self.intersections[id]= (x,y,z)

    def GetNextIntersectionID(self):
        _intersectionIDs = self.intersections.keys()
        return _intersectionIDs[0]

    def UpdateTrafficState(self,msg):

        if msg.aircraftID in self.trafficTraj.keys():
            self.trafficTraj[msg.aircraftID].append(msg.position)
        else:
            self.trafficTraj[msg.aircraftID] = []
            self.trafficTraj[msg.aircraftID].append(msg.position)

        #TODO: check if you really need this log update
        if msg.aircraftID == "vehicle2":
            self.ownship.UpdateLog(self.trafficTraj)

    def UpdateBlackBoxLog(self):
        position = (self.ownship.x,self.ownship.y)
        velocity = (self.ownship.vx,self.ownship.vy)
        entry = [time.time(),self.server._state.name,len(self.server._log),position[0],position[1],velocity[0],velocity[1]]
        self.blacbox.append(entry)

    def ComputeDistance(self,A,B):
        return np.sqrt((A[0] -B[0])**2 + (A[1] -B[1])**2 + (A[2] -B[2])**2 )

    def DetermineCrossingTime(self,id,t):
        """
        Determine crossing time to the next intersection
        :param intersection: [x,y,z] coordinates of intersection
        :return: [r,d] release time and deadline
        """

        nextIntersection = self.intersections[id]
        dist2zone = self.ComputeDistance((self.ownship.x,self.ownship.y,self.ownship.z),nextIntersection)
        hypo    = np.sqrt(dist2zone**2 + self.xtrackdev**2)
        speed = np.sqrt(self.ownship.vx ** 2 + self.ownship.vy ** 2 + self.ownship.vz ** 2)

        A = (self.ownship.x, self.ownship.y,0)
        B = nextIntersection
        AB = (B[0] - A[0], B[1] - A[1])
        Vel2 = (self.ownship.vx,self.ownship.vy)
        proj = AB[0]*Vel2[0] + AB[1]*Vel2[1]

        if proj < 0 and dist2zone < 3:
            return False

        maxdist = self.xtrackdev + hypo
        mindist = dist2zone

        minT    = mindist/(self.ownship.vmax-2)
        maxT    = maxdist/self.ownship.vmin
        release = minT+t
        deadline = maxT+t
        reach   = mindist/self.speed + t
        if len(self.trajectory) > 0:
            reach = t + self.Time2FollowTrajectory(A, self.trajectory)

        if id not in self.crossingTimes.keys():
            self.crossingTimes[id] = {}

        self.crossingTimes[id] = [release, reach, deadline]

        return True

    def BroadcastCurrentPosition(self):
        msg = acState_t()
        msg.aircraftID  = self.ownship.id
        msg.timestamp = int(time.time())
        msg.position[0] = self.ownship.x
        msg.position[1] = self.ownship.y
        msg.position[2] = self.ownship.z
        msg.velocity[0] = self.ownship.vx
        msg.velocity[1] = self.ownship.vy
        msg.velocity[2] = self.ownship.vz

        self.server.threadLock.acquire()
        self.lcm.publish("POSITION",msg.encode())
        self.server.threadLock.release()

    def ComputeSchedule(self,log):
        """
        :param log: log from RAFT server containing job data
        :return:  compute schedule for all aircraft in log data

        A schedule is computed using the PolynomialTime algorithm
        as described in "Efficient Algorithms for Collision Avoidance
        at intersections", Colombo et al.
        """
        print "Computing schedule"

        _jobData = {}

        count = 1
        computeSchedule = False
        while(count <= len(log)):
            element = log[-count]
            count += 1
            if element["data"][0] == 1 and count > 2:
                count = len(log) + 1
                continue
            elif element["data"][0] == 1:
                intersectionID = element["data"][1]
                vehicleID = element["data"][2]
                release = element["data"][3]
                deadline = element["data"][4]
                currentTime = element["data"][5]
                _jobData[vehicleID] = [release, currentTime, deadline]
                if vehicleID == self.server._name:
                    computeSchedule = True

        if not computeSchedule:
            print "Ownship not found in log"
            return False

        # ids of all aircraft attempting to cross intersection id
        acid = [i for i in _jobData.keys()]
        R = []
        D = []
        for e in acid:
            R.append(_jobData[e][0]/self.delta)
            D.append(_jobData[e][2]/self.delta + 1)

        T, sortedJ, status = PolynomialTime(R, D)

        print R,D,T

        for i,elem in enumerate(sortedJ):
            index = elem[0]
            name  = acid[index]
            self.schedules[name] = T[i]*self.delta

        return True

    def ComputeSpeed(self,D,t):
        """
        ComputeSpeed performs a line search to determine
        speed \in [vmin,vmax] while minimizing crosstrack
        deviation within [0,xmax]. Minimization is done
        approximately.
        :param D: Straight line distance to next intersection
        :param t: Scheduled crossing time
        :return: return flight speed and xtrack deviation
        """

        getV = lambda x: (x + np.sqrt(D**2 + x**2))/t

        X = [i for i in np.arange(0,self.xtrackdev+5,5)]
        v = 0
        for x in X:
            v = getV(x)
            if self.ownship.vmin <= v <= self.ownship.vmax:
                self.speed = v
                return x,v
            else:
                continue

        return -1,-1

    def ComputeTrajectory(self,id,T):

        A  = (self.ownship.x, self.ownship.y)
        B  = self.intersections[id]
        AB = (B[0] - A[0], B[1] - A[1])
        distAB = np.sqrt(AB[0] ** 2 + AB[1] ** 2)
        print "Schedule T:" + str(T)
        print "Current Time:" + str(time.time())
        T = T - time.time()
        (x,s) = self.ComputeSpeed(distAB,T)
        print x,s

        fac = x/distAB
        AC = [AB[1]*fac,-AB[0]*fac]
        C = (AC[0] + A[0],AC[1] + A[1],0)

        print AC,C,B
        self.trajectory = []
        self.trajectory.append(C)
        self.trajectory.append(B)
        return s

    def Time2FollowTrajectory(self,position,trajectory):

        traj = [position]  + trajectory
        dist = 0
        for i,element in enumerate(traj):
            if i < len(traj)-1:
                dist += self.ComputeDistance(traj[i],traj[i+1])

        time = dist/self.speed
        return time

    def FollowTrajectory(self,trajectory):

        currentPos = (self.ownship.x,self.ownship.y,self.ownship.z)

        if len(trajectory) > 0:
            nextPos = trajectory[0]
        else:
            return

        dist = self.ComputeDistance(currentPos,nextPos)
        if dist < 5:
            trajectory.pop(0)
            if len(trajectory) > 0:
                print "next waypoint"
                nextPos = trajectory[0]
                print nextPos
            else:
                return

        vec = (nextPos[0] - currentPos[0],nextPos[1] - currentPos[1],nextPos[2] - currentPos[2])
        normVec = np.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)
        fac = self.speed/normVec
        controlVec = (vec[0]*fac,vec[1]*fac,vec[2]*fac)
        self.ownship.UpdateControl(controlVec[0],controlVec[1],controlVec[2])

    def CheckConflicts(self,log,connectedServers):

        entryTime = {}
        count = 1
        while (count <= len(log)):
            element = log[-count]
            count += 1
            if element["data"][0] == 1 and count >= 2:
                count = len(log) + 1
                continue
            elif element["data"][0] == 0:
                intersectionID = element["data"][1]
                vehicleID = element["data"][2]
                release = element["data"][3]
                deadline = element["data"][4]
                currentTime = element["data"][5]
                entryTime[vehicleID] = currentTime

        if len(entryTime.keys()) < len(connectedServers):
            # Not account for conflicts if all data is not available
            return False

        # Sort dictionary item based on time of entry
        entryTime = sorted(entryTime.iteritems(),key=lambda (x,v):(v,x))
        entryTimeList = [val[1] for val in entryTime]
        for i in range(1,len(entryTimeList)):
            diff = entryTimeList[i] - entryTimeList[i-1]
            # TODO: analyse interaction between separation distance and this conflict trigger
            if diff < self.delta*0.5:
                return True

        return False

    def GetPrevCrossingTimeFromLog(self):
        log = self.server.get_log()
        log.reverse()
        if len(log) > 0:
            for i,element in enumerate(log):
                if element["data"][0] == 1 and i > 0:
                    #print "couldn't find previous crossing time"
                    return 0
                if element["data"][0] == 0 and np.fabs(element["data"][2] - self.ownship.id) < 1e-3:
                    return element["data"][5]

        return 0


    def run(self):
        while not self.stopped():
            t1 = time.time()
            if t1 - self.pt0 >= self.ownship.dt:
                self.UpdateBlackBoxLog()
                self.ownship.dt = t1 - self.pt0
                self.pt0 = t1
                self.FollowTrajectory(self.trajectory)
                self.ownship.UpdateState()
                self.ownship.dt = 0.1
                self.BroadcastCurrentPosition()

                if self.server._shutdown:
                    self.server._state.name = "NEUTRAL"
                    continue

                # if crossing time not available previously, or if
                # crossing times have changed from previous values,
                # recompute and broadcast that information to the leader
                nextin = self.GetNextIntersectionID()
                status = self.DetermineCrossingTime(nextin,t1)

                if status == False:
                    if not self.shutdownSent:
                        self.shutdownSent = True
                        if self.server._leader is not None:
                            self.server.leave_network()
                            print "sent shutdown request"
                            if type(self.server._state) is Follower:
                                print "expective response from leader"
                                self.server._expectResponse = True
                    continue

                _xtime = self.crossingTimes[nextin][1]
                prevCrossingT = self.GetPrevCrossingTimeFromLog()

                log = self.server.get_log()
                #NOTE: hack to ensure I don't spam the server with client messages until my log gets populated
                if self._atLeastOneMsgSent and len(log) < 1:
                    prevCrossingT = _xtime

                if abs(prevCrossingT - _xtime) > 2:
                    job = client_status_t()
                    job.data.append(0)
                    job.data.append( nextin)
                    job.data.append( self.ownship.id)
                    job.data.append( self.crossingTimes[nextin][0])
                    job.data.append( self.crossingTimes[nextin][2])
                    job.data.append( _xtime)
                    job.n = 6
                    if self.server._leader is not None:
                        self.lcm.publish(self.server._leader+"_CLIENT_STATUS",job.encode())
                        self._atLeastOneMsgSent = True


            # if self node is leader
            # Go through all the logs in the server and check if
            # entries maintain separation distance. If entries don't
            # require separation, send command to compute schedule.
            log = self.server.get_log()
            if type(self.server._state) is Leader:
                log = self.server.get_log()
                commitedlog = log[:self.server._commitIndex]
                val = self.CheckConflicts(commitedlog,self.server._connectedServers)
                if val is True and not self.computeSent:
                    job = client_status_t()
                    job.data.append(1)
                    job.n = 1
                    if self.server._leader is not None:
                        self.lcm.publish(self.server._leader+"_CLIENT_STATUS",job.encode())

                    self.computeSent = True

            N = 0
            if type(self.server._state) is Leader:
                N = self.server._commitIndex
            else:
                N = len(log)

            executeCommand = False
            entry = None
            for i in range(self.lastProcessedIndex,N):
                if N > 0:
                    nextIndex2Process = self.lastProcessedIndex + 1
                    self.lastProcessedIndex = nextIndex2Process
                    entry = log[nextIndex2Process - 1]

                    if entry["data"] == 1:
                        executeCommand = True
                        self.computeSent = False
                        break

            if executeCommand:
                print entry
                print "executing command entry"
                # After executing command, clear logs.
                #self.server.clear_log()

                # Go through the log and extract [r,d] and current t information.
                self.newSchedule = self.ComputeSchedule(log[:N])
                self.lastLogLength = 0

            if self.newSchedule:
                self.newSchedule = False
                nextIntersection = self.intersections.keys()[0]
                intersectionT = self.schedules[self.ownship.id]
                self.speed = self.ComputeTrajectory(nextIntersection,intersectionT)

        print "Exiting thread"

    # LCM handlers:
    def HandleTrafficPosition(self,channel, data):
        msg = acState_t.decode(data)
        if msg.aircraftID != self.ownship.id:
            self.UpdateTrafficState(msg)