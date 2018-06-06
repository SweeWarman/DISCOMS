import csv
from math import sqrt, ceil
from Animation import AgentAnimation
from matplotlib import pyplot as plt
import sys

def Interpolate(t1,x1,y1,t2,x2,y2,t):
    x = x1 + (x2-x1)/(t2-t1) * (t-t1)
    y = y1 + (y2-y1)/(t2-t1) * (t-t1)
    return (x,y)

def ComputeDistance(A,B):
    return sqrt((A[0] -B[0])**2 + (A[1] -B[1])**2)# + (A[2] -B[2])**2 )

def FindIndex(t0,t):

    if len(t) == 2:
        index = 0
        return index

    half = int(ceil(float(len(t))/2))

    if t[half-1] >= t0:
        index = FindIndex(t0,t[:half])
    else:
        index = half-1 + FindIndex(t0,t[half-1:])

    return index

def FindInterpolatedValue(t0,data,t):

    if t0 <= t[0]:
        index = 0
    elif t0 >= t[-1]:
        index = len(t)-2
    else:
        index = FindIndex(t0,t)

    x1 = data[index][0]
    y1 = data[index][1]
    x2 = data[index+1][0]
    y2 = data[index+1][1]
    t1 = t[index]
    t2 = t[index+1]

    (xp,yp) = Interpolate(t1,x1,y1,t2,x2,y2,t0)

    return (xp,yp)

def GetStateId(name):
    if name == "NEUTRAL":
        return 0
    elif name == "FOLLOWER":
        return 1
    elif name == "CANDIDATE":
        return 2
    elif name == "LEADER":
        return 3

numVehicles = int(sys.argv[1])

Data = {}
InterpPosition= {}
InterpVelocity={}
InterpDist2Int = {}
InterpData = {}
dataTimes = None
nameList = []

for i in range(1,numVehicles+1):
    nameList.append("NODE"+str(i))

for name in nameList:
    filename = name + '.csv'
    with open(filename, 'rb') as csvfile:
         logreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
         Data[name] = {}
         Data[name]["t"] = []
         Data[name]["state"] = []
         Data[name]["loglen"] = []
         Data[name]["position"] = []
         Data[name]["velocity"] = []
         Data[name]["dist2int"] = []
         for i,row in enumerate(logreader):
             Data[name]["t"].append(float(row[0]))
             Data[name]["state"].append(row[1])
             Data[name]["loglen"].append(int(row[2]))
             position = (float(row[3]),float(row[4]))
             Data[name]["position"].append(position)
             Data[name]["velocity"].append((float(row[5]),float(row[6])))
             Data[name]["dist2int"].append(ComputeDistance(position,(0,0)))
         if name == 'NODE1':
             dataTimes = Data[name]["t"]
             InterpPosition[name] = Data[name]["position"]
             InterpVelocity[name] = Data[name]["velocity"]
         else:
             InterpPosition[name] = []
             InterpVelocity[name] = []
             for t0 in dataTimes:
                 position = FindInterpolatedValue(t0,Data[name]["position"],Data[name]["t"])
                 velocity = FindInterpolatedValue(t0,Data[name]["velocity"],Data[name]["t"])
                 InterpPosition[name].append(position)
                 InterpVelocity[name].append(velocity)

         InterpData[name] = {}
         InterpData[name]["position"] = InterpPosition[name]
         InterpData[name]["velocity"] = InterpVelocity[name]

plt.figure(1)
for i in range(1,numVehicles+1):
    name = "NODE"+str(i)
    plt.plot(Data[name]["t"],Data[name]["dist2int"],label=name)
plt.xlabel("time (s)")
plt.ylabel("Distance to Intersection (m)")
plt.legend()
plt.rc('text',usetex=True)
plt.rc('font',family='serif')

plt.figure(2)
for name in nameList:
    stateid = [GetStateId(state) for state in Data[name]["state"]]
    t = Data[name]["t"]
    plt.plot(t,stateid,label=name)

plt.ylim([-1,4])
plt.yticks((0,1,2,3),("NEUTRAL","FOLLOWER","CANDIDATE","LEADER"))
plt.xlabel("time (s)")
plt.ylabel("Raft state")
plt.legend()
plt.rc('text',usetex=True)
plt.rc('font',family='serif')


maxplayback = len(Data["NODE1"]["t"])
anim = AgentAnimation(-200,-200,200,200)
color = ['b','g','r','m','y','k']
for i in range(1,numVehicles+1):
    name = "NODE"+str(i)
    anim.AddAgent(name,2.5,color[i-1])

anim.AddData(InterpData,maxplayback)
anim.run()
