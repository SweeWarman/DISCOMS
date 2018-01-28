import matplotlib.pyplot as plt
import matplotlib.animation as animation

class AgentAnimation():
    def __init__(self,xmin,ymin,xmax,ymax):
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(xmin, xmax), ylim=(ymin, ymax))
        self.agents = []
        self.data = None

    def AddAgent(self,radius,color):
        agt = plt.Circle((0.0, 0.0), radius=radius, fc=color)
        self.ax.add_patch(agt)
        self.agents.append(agt)

    def AddData(self,data):
        self.data = data
        self.lengths = [len(entry) for entry in self.data.values()]
        self.minlen = min(self.lengths)

    def init(self):
        return self.agents

    def animate(self,i):
        if i < self.minlen:
            for j, vehicle in enumerate(self.agents):
                id = "vehicle" + str(j+1)
                vehicle.center = (self.data[id][i][0], self.data[id][i][1])
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


