import csv
from math import sqrt, ceil

def Interpolate(t1,x1,y1,t2,x2,y2,t):
    x = x1 + (x2-x1)/(t2-t1) * (t-t1)
    y = y1 + (y2-y1)/(t2-t1) * (t-t1)
    return (x,y)

def ComputeDistance(self,A,B):
    return sqrt((A[0] -B[0])**2 + (A[1] -B[1])**2 + (A[2] -B[2])**2 )
"""
Data = {}

name = "vehicle1"
with open('vehicle1.csv', 'rb') as csvfile:
     logreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
     Data[name] = {}
     Data[name]["t"] = []
     Data[name]["state"] = []
     Data[name]["loglen"] = []
     Data[name]["position"] = []
     Data[name]["velocity"] = []
     for row in logreader:
         Data[name]["t"].append(float(row[0]))
         Data[name]["state"].append(row[1])
         Data[name]["loglen"].append(int(row[2]))
         Data[name]["position"].append((float(row[3]),float(row[4])))
         Data[name]["velocity"].append((float(row[5]),float(row[6])))

dataTimes = Data["vehicle1"]["t"]

interpData = {}
"""

def FindIndex(t0,t):

    if len(t0) == 2:
        index = 0
        return index

    half = int(ceil(float(len(t0))/2))

    if t0[half-1] >= t:
        index = FindIndex(t0[:half],t)
    else:
        index = half-1 + FindIndex(t0[half-1:],t)

    return index

def FindInterpolatedValue(t0,data,t):

    if t[0] <= t0:
        index = 0
    elif t[-1] >= t0:
        index = len(t)-2
    else:
        index = FindIndex(t0,t)

    x1 = data[index][0]
    y1 = data[index][1]
    x2 = data[index][0]
    y2 = data[index][1]

A = [i*0.5 for i in range(10)]

index = FindIndex(A,1.3)

print index