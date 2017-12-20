import numpy as np
import threading
from msg import acState_t
from msg import jobprop_t
from msg import xtimerequest_t
from PolynomialTime import PolynomialTime
import time

class UAVAgent(threading.Thread):
    def __init__(self,id,x,y,z,vx,vy,vz,lc,log):
        threading.Thread.__init__(self)
        self.threadLock = threading.Lock()
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.dt = 0.1
        self.traffic = []
        self.trafficTraj = {}
        self.lc = lc
        self.log = log
        self.trajx = []
        self.trajy = []
        self.trajz = []
        self.pt0 = time.time()
        self.bt0 = time.time()
        self.status = True
        self.intersections = {}
        self.crossingTimes = {}
        self.vmax = 10
        self.vmin = 5
        self.xtrackdev = 20
        self.schedules = {}
        self.newSchedule = False
        self.delta = 5;
        self.vx0 = vx
        self.vy0 = vy
        self.vz0 = vz
        self.trajectory = []
        self.speed = np.sqrt(vx**2 + vy**2 + vz**2)

    def AddIntersections(self,id,x,y,z):
        self.intersections[id]= (x,y,z)

    def UpdateState(self):
        self.x = self.x + self.vx * self.dt
        self.y = self.y + self.vy * self.dt
        self.z = self.z + self.vz * self.dt

    def UpdateControl(self,vx,vy,vz):
        self.threadLock.acquire()
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.threadLock.release()

    def UpdateTrafficState(self,msg):
        print "Updating traffic info"
        traffic = {"id":msg.aircraftID,"pos":msg.position,"vel":msg.velocity}
        if msg.aircraftID in self.trafficTraj.keys():
            self.trafficTraj[msg.aircraftID].append(msg.position)
        else:
            self.trafficTraj[msg.aircraftID] = []
            self.trafficTraj[msg.aircraftID].append(msg.position)

            req = xtimerequest_t()
            req.fromAircraftID = self.id
            req.toAircraftID   = msg.aircraftID
            req.intersectionID = self.intersections.keys()[0]
            print "requesting self crossing time"
            self.lc.publish("REQUEST", req.encode())

        if self.log:
            self.trajx.append(self.x)
            self.trajy.append(self.y)
            self.trajz.append(self.z)

    def ComputeDistance(self,A,B):
        return np.sqrt((A[0] -B[0])**2 + (A[1] -B[1])**2 + (A[2] -B[2])**2 )

    def DetermineCrossingTime(self,id):
        """
        Determine crossing time to the next intersection
        :param intersection: [x,y,z] coordinates of intersection
        :return: [r,d] release time and deadline
        """
        nextIntersection = self.intersections[id]
        mindist = self.ComputeDistance((self.x,self.y,self.z),nextIntersection)
        hypo    = np.sqrt(mindist**2 + self.xtrackdev**2)
        maxdist = self.xtrackdev + hypo
        minT    = mindist/self.vmax
        maxT    = maxdist/self.vmin
        t       = time.time()
        release = minT+t
        deadline = maxT+t
        if id not in self.crossingTimes.keys():
            self.crossingTimes[id] = {}

        self.crossingTimes[id][self.id] = [release, deadline]

    def ProcessRequest(self,msg):

        reqIntID = msg.intersectionID

        if reqIntID in self.crossingTimes.keys():
            reply = jobprop_t()
            reply.aircraftID = self.id
            reply.intersectionID = reqIntID
            reply.release = self.crossingTimes[reqIntID][self.id][0]
            reply.deadline = self.crossingTimes[reqIntID][self.id][1]
            print "replying to request"
            self.lc.publish("JOB", reply.encode())


    def UpdateIntruderCrossingTime(self,msg):
        print "Updating intersection crossing times"
        if msg.intersectionID not in self.crossingTimes.keys():
            self.crossingTimes[msg.intersectionID] = {}

        if msg.aircraftID not in self.crossingTimes[msg.intersectionID].keys():
            self.crossingTimes[msg.intersectionID][msg.aircraftID] = []

        self.crossingTimes[msg.intersectionID][msg.aircraftID] = [msg.release,msg.deadline]
        self.newSchedule = True

    def BroadcastCurrentPosition(self):
        msg = acState_t()
        msg.aircraftID  = self.id
        msg.position[0] = self.x
        msg.position[1] = self.y
        msg.position[2] = self.z
        msg.velocity[0] = self.vx
        msg.velocity[1] = self.vy
        msg.velocity[2] = self.vz
        self.lc.publish("POSITION",msg.encode())

    def ComputeSchedule(self,id):
        print "Computing schedule"
        # ids of all aircraft attempting to cross intersection id
        acid = [i for i in self.crossingTimes[id].keys()]
        R = []
        D = []
        for e in acid:
            R.append(self.crossingTimes[id][e][0]/self.delta)
            D.append(self.crossingTimes[id][e][1]/self.delta + 1)

        T, sortedJ, status = PolynomialTime(acid, R, D)

        if self.id not in self.schedules.keys():
            self.schedules[self.id] = {}
        for i,elem in enumerate(sortedJ):
            self.schedules[self.id][elem[0]] = T[i]*self.delta


    def CollectThirdpartySchedules(self,msg):
        if msg.aircraftID not in self.schedules.keys():
            self.schedules[msg.aircraftID] = {}

        for i in range(msg.numAircrafts):
            self.schedules[msg.aircraftID][msg.ids[i]] = msg.T[i]

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

        X = [i for i in np.arange(0,self.xtrackdev,5)]
        v = 0
        for x in X:
            v = getV(x)
            if self.vmin <= v <= self.vmax:
                self.speed = v
                return x,v
            else:
                continue

        return -1,-1

    def ComputeTrajectory(self,T,id):

        A  = (self.x, self.y)
        B  = self.intersections[id]
        AB = (B[0] - A[0], B[1] - A[1])
        distAB = np.sqrt(AB[0] ** 2 + AB[1] ** 2)

        (x,s) = self.ComputeSpeed(distAB,T)
        hypo = np.sqrt(distAB**2 + x**2)
        fac = hypo/distAB
        AC = [AB[1]*fac -AB[0]*fac]
        C = (AC[0] + A[0],AC[1] + A[1])

        self.trajectory.append(C)
        self.trajectory.append(B)
        return s

    def FollowTrajectory(self,trajectory):

        currentPos = (self.x,self.y,self.z)

        if len(trajectory) > 0:
            nextPos = trajectory[0]
        else:
            self.UpdateControl(self.vx0,self.vy0,self.vz0)
            return

        dist = self.ComputeDistance(currentPos,nextPos)
        if dist < 2.5:
            trajectory.pop(0)
            if len(trajectory) > 0:
                nextPos = trajectory[0]
            else:
                self.UpdateControl(self.vx0, self.vy0, self.vz0)
                return

        vec = (nextPos[0] - currentPos[0],nextPos[1] - currentPos[1],nextPos[2] - currentPos[2])
        normVec = np.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)
        fac = self.speed/normVec
        controlVec = (vec[0]*fac,vec[1]*fac,vec[2]*fac)
        self.UpdateControl(controlVec[0],controlVec[1],controlVec[2])


    def run(self):
        while self.status:
            t1 = time.time()
            if t1 - self.pt0 >= self.dt:
                self.pt0 = t1
                self.FollowTrajectory(self.trajectory)
                self.UpdateState()

            if t1 - self.bt0 > 0.5:
                self.bt0 = t1
                self.BroadcastCurrentPosition()

            for k in self.intersections.keys():
                if k not in self.crossingTimes.keys():
                    self.DetermineCrossingTime(k)
                elif self.id not in self.crossingTimes[k].keys():
                    self.DetermineCrossingTime(k)

            if self.newSchedule:
                self.ComputeSchedule(0)
                self.newSchedule = False
                nextIntersection = self.intersections.keys()[0]
                intersectionT = self.schedules[self.id][self.id]
                self.ComputeTrajectory()

        print "Exiting thread"
