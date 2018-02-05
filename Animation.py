import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import sin,cos,atan2

class AgentAnimation():
    def __init__(self,xmin,ymin,xmax,ymax):
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(xmin, xmax), ylim=(ymin, ymax))
        self.agents = []
        self.agentNames = []
        self.data = None

    def AddAgent(self,name,radius,color):
        #agt = plt.Circle((0.0, 0.0), radius=radius, fc=color)
        agt = self.GetTriangle((0.0,0,0),(1.0,0.0),color)
        self.ax.add_patch(agt)
        self.agents.append(agt)
        self.agentNames.append(name)

    def GetTriangle(self,pos, vel, col):
        x = pos[0]
        y = pos[1]

        t = atan2(vel[1],vel[0])

        x1 = x + 2 * cos(t)
        y1 = y + 2 * sin(t)

        tempX = x - 1 * cos(t)
        tempY = y - 1 * sin(t)

        x2 = tempX + 1 * cos((t + 90))
        y2 = tempY + 1 * sin((t + 90))

        x3 = tempX - 1 * cos((t + 90))
        y3 = tempY - 1 * sin((t + 90))

        return plt.Polygon([[x1, y1], [x2, y2], [x3, y3]], color=col, fill=True)

    def UpdateTriangle(self,pos, vel, poly):
        x = pos[0]
        y = pos[1]

        t = atan2(vel[1], vel[0])

        x1 = x + 2 * cos(t)
        y1 = y + 2 * sin(t)

        tempX = x - 1 * cos(t)
        tempY = y - 1 * sin(t)

        x2 = tempX + 1 * cos((t + 90))
        y2 = tempY + 1 * sin((t + 90))

        x3 = tempX - 1 * cos((t + 90))
        y3 = tempY - 1 * sin((t + 90))

        poly.set_xy([[x1, y1], [x2, y2], [x3, y3]])

    def AddData(self,data,maxplayback):
        self.data = data
        self.minlen = maxplayback

    def init(self):
        return self.agents

    def animate(self,i):
        if i < self.minlen:
            for j, vehicle in enumerate(self.agents):
                id = self.agentNames[j]
                #vehicle.center = (self.data[id][i][0], self.data[id][i][1])
                position = (self.data[id]["position"][i][0], self.data[id]["position"][i][1])
                velocity = (self.data[id]["velocity"][i][0], self.data[id]["velocity"][i][1])
                self.UpdateTriangle(position,velocity,vehicle)
        return self.agents

    def run(self):
        animate = lambda x: self.animate(x)
        init = lambda:self.init()
        self.anim = animation.FuncAnimation(self.fig, animate,
                                       init_func=init,
                                       frames=1000,
                                       interval=150,
                                       blit=False)

        plt.show()