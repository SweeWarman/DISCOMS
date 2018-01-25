import numpy as np
import threading
from exlcm import acState_t,jobprop_t
from PolynomialTime import PolynomialTime
from lcmraft.states.leader import Leader
from lcmraft.states.state import EntryType
from lcmraft.LcmRaftMessages import client_status_t
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
        self.xtrackdev = 20
        self.schedules = {}
        self.newSchedule = False
        self.delta = 5
        self.trajectory = []
        self.prevCrossingT = 0
        self.speed = np.sqrt(self.ownship.vx**2 + self.ownship.vy**2 + self.ownship.vz**2)
        self.zone = 50
        self.server = None
        self.lastLogLength = 0

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

        self.ownship.UpdateLog()

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
        maxdist1 = 0
        if len(self.trajectory) > 0:
            for wp in self.trajectory:
                maxdist1 += self.ComputeDistance((self.ownship.x,self.ownship.y,self.ownship.z),wp)
        else:
            maxdist1 = dist2zone

        maxdist2 = self.xtrackdev + hypo

        mindist = dist2zone
        if abs(mindist - maxdist1) < 1e-1:
            maxdist = maxdist2
        else:
            maxdist = maxdist1

        minT    = mindist/(self.ownship.vmax-2)
        maxT    = maxdist/self.ownship.vmin
        release = minT+t
        deadline = maxT+t
        reach   = mindist/speed + t
        #print "Crossing Time:"
        #print minT,maxT,t,release,reach,deadline
        #print mindist/speed,t,speed,self.ownship.x
        if id not in self.crossingTimes.keys():
            self.crossingTimes[id] = {}

        self.crossingTimes[id][self.ownship.id] = [release, reach, deadline]

    def BroadcastCurrentPosition(self):
        msg = acState_t()
        msg.aircraftID  = self.ownship.id
        msg.position[0] = self.ownship.x
        msg.position[1] = self.ownship.y
        msg.position[2] = self.ownship.z
        msg.velocity[0] = self.ownship.vx
        msg.velocity[1] = self.ownship.vy
        msg.velocity[2] = self.ownship.vz

        self.server.threadLock.acquire()
        self.lcm.publish("POSITION",msg.encode())
        self.server.threadLock.release()

    def ComputeSchedule(self,id):
        print "Computing schedule"
        # ids of all aircraft attempting to cross intersection id
        acid = [i for i in self.crossingTimes[id].keys()]
        R = []
        D = []
        for e in acid:
            R.append(self.crossingTimes[id][e][0]/self.delta)
            D.append(self.crossingTimes[id][e][2]/self.delta + 1)

        #TODO: clean out this index manipulation
        _airplaneIds = [int(val[7])-1 for val in acid]

        T, sortedJ, status = PolynomialTime(_airplaneIds, R, D)

        print R,D,T

        if self.ownship.id not in self.schedules.keys():
            self.schedules[self.ownship.id] = {}
        for i,elem in enumerate(sortedJ):
            #TODO: clean out this index manipulation
            index = elem[0] + 1
            name  = "vehicle"+str(index)
            self.schedules[self.ownship.id][name] = T[i]*self.delta

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
        self.trajectory.append(C)
        self.trajectory.append(B)
        return s

    def FollowTrajectory(self,trajectory):

        currentPos = (self.ownship.x,self.ownship.y,self.ownship.z)

        if len(trajectory) > 0:
            nextPos = trajectory[0]
        else:
            return

        dist = self.ComputeDistance(currentPos,nextPos)
        if dist < 2.5:
            trajectory.pop(0)
            if len(trajectory) > 0:
                nextPos = trajectory[0]
            else:
                return

        vec = (nextPos[0] - currentPos[0],nextPos[1] - currentPos[1],nextPos[2] - currentPos[2])
        normVec = np.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)
        fac = self.speed/normVec
        controlVec = (vec[0]*fac,vec[1]*fac,vec[2]*fac)
        self.ownship.UpdateControl(controlVec[0],controlVec[1],controlVec[2])

    def CheckConflicts(self,log,connectedServers):

        entryTime = []
        for element in log:
            if element["entryType"] == EntryType.DATA.value:
                intersectionID = element["intersectionID"]
                vehicleID = element["vehicleID"]
                release = element["entryTime"]
                deadline = element["exitTime"]
                currentTime = element["crossingTime"]
                entryTime.append(currentTime)
                self.crossingTimes[intersectionID][vehicleID] = [release,currentTime,deadline]

        entryTime.sort()

        for i in range(1,len(entryTime)):
            diff = entryTime[i] - entryTime[i-1]

            if diff < self.delta:
                return True

        return False

    def run(self):
        while self.status:
            t1 = time.time()
            if t1 - self.pt0 >= self.ownship.dt:
                self.ownship.dt = t1 - self.pt0
                self.pt0 = t1
                self.FollowTrajectory(self.trajectory)
                self.ownship.UpdateState()
                self.ownship.dt = 0.1
                self.BroadcastCurrentPosition()

                # if crossing time not available previously, or if
                # crossing times have changed from previous values,
                # recompute and broadcast that information to the leader
                nextin = self.GetNextIntersectionID()
                self.DetermineCrossingTime(nextin,t1)
                _xtime = self.crossingTimes[nextin][self.ownship.id][1]
                if abs(self.prevCrossingT - _xtime) > 2:
                    job = client_status_t()
                    job.intersectionID = nextin
                    job.entryTime = self.crossingTimes[nextin][self.ownship.id][0]
                    job.exitTime = self.crossingTimes[nextin][self.ownship.id][2]
                    job.crossingTime = _xtime
                    job.vehicleID = self.ownship.id

                    if self.server._leader is not None:
                        self.lcm.publish("CLIENT_STATUS",job.encode())
                        self.prevCrossingT = _xtime

            # if self node is leader
            # Go through all the logs in the server and check if
            # entries maintain separation distance. If entries don't
            # require separation, send command to compute schedule.

            entry = self.server.get_last_commited_log_entry()

            if type(self.server._state) is Leader:
                log = self.server.get_log()

                matches = 0
                if len(log) > 0:
                    for node in self.server._connectedServers:
                        for entry in log:
                            if entry["entryType"] == EntryType.DATA.value:
                                if node == entry["vehicleID"]:
                                    matches += 1

                if matches >= len(self.server._connectedServers) and len(log) != self.lastLogLength:
                    val = self.CheckConflicts(log,self.server._connectedServers)

                    if val is True and entry["entryType"] != EntryType.COMMAND.value:
                        self.server._state.SendComputeCommand(0)
                        self.lastLogLength = len(log)

            # If command entry found in log, execute or:
            if (entry is not None):
                if entry["entryType"] == EntryType.COMMAND.value:
                    executeCommand = False

                    if type(self.server._state) is Leader:
                        if self.server._commitIndex == len(self.server._log):
                            executeCommand = True
                    else:
                        executeCommand = True

                    if executeCommand:
                        print "executing command entry"
                        log = self.server.get_log()
                        # After executing command, clear logs.
                        self.server.clear_log()

                        # Go through the log and extract [r,d] and current t information.
                        self.ComputeSchedule(0)
                        self.newSchedule = True
                        self.lastLogLength = 0

            if self.newSchedule:
                self.newSchedule = False
                nextIntersection = self.intersections.keys()[0]
                intersectionT = self.schedules[self.ownship.id][self.ownship.id]
                self.ComputeTrajectory(nextIntersection,intersectionT)

        print "Exiting thread"

    # LCM handlers:
    def HandleTrafficPosition(self,channel, data):
        msg = acState_t.decode(data)
        if msg.aircraftID != self.ownship.id:
            self.UpdateTrafficState(msg)