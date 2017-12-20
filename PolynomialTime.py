import numpy as np

def within(x,F,FK,closed):

    if closed is True:
        for f in F:
            if len(f) == 2:
                if (x >= f[0]) and (x <= f[1]):
                    FK[:] = f
                    return True
    else:
        for f in F:
            if len(f) == 2:
                if (x > f[0]) and (x < f[1]):
                    FK[:] = f
                    return True

    return False

def LeastDeadlineAtT(Jobs,t,ScheduledJobs):
    Deadlines=[]
    for J in Jobs:
        if J[1] <= t:
            Deadlines.append(J)

    sortedD = sorted(Deadlines,key = lambda x:x[2])
    for J in sortedD:
        if J[0] in ScheduledJobs:
            continue
        else:
            return J

    return []

def PolynomialTime(id,R,D):
    n = len(R)
    F = []
    c = []
    J = zip(id,R,D)
    sortedJ = sorted(J,key=lambda x:x[1])
    id,R,D = zip(*sortedJ)
    for i in range(n):
        F.append([])
        c.append(1e10)

    for i in range(n-1,-1,-1):
        for j in range(0,n,1):
            if D[j] >= D[i]:
                if c[j] is 1e10:
                    c[j] = D[j] - 1
                else:
                    c[j] = c[j] - 1

            Fk = []
            while within(c[j],F,Fk,False):
                c[j] = min(Fk)

        if i==0:
            C = min(c)
            if C < R[i]:
                return [],[],False

            Fk = []
            if within(C,[[R[i] + R[i]+1]],Fk,True):
                F[i] = [C-1,R[i]]

        elif R[i-1] < R[i]:
            C = min(c)
            if C < R[i]:
                print "WARNING: NO SCHEDULE FOUND"
                return ([], False)

            Fk = []
            if within(C, [[R[i],R[i] + 1]],Fk,True):
                F[i] = [C - 1, R[i]]

    t = 0
    ScheduledJobs = []
    Rc = R[:]
    T = [0 for i in range(n)]
    for i in range(n):
        rmin = min([r[1] for r in sortedJ if r[0] not in ScheduledJobs])
        arg_rmin = np.argmin(Rc)
        t = max([t,rmin])
        Fk = []
        while within(t,F,Fk,False):
            t = max(Fk)

        minDJob = LeastDeadlineAtT(sortedJ,t,ScheduledJobs)
        j = minDJob[0]
        T[j] = t
        t = t+1
        ScheduledJobs.append(j)

    return T,sortedJ,True

