
class Vehicle:
    def __init__(self, id, position, velocity):
        self.id = id
        self.x = position[0]
        self.y = position[1]
        self.z = position[2]
        self.vx = velocity[0]
        self.vy = velocity[1]
        self.vz = velocity[2]
        self.traj = []
        self.vmax = 10
        self.vmin = 4
        self.dt = 0.1

    def UpdateState(self):
        self.x = self.x + self.vx * self.dt
        self.y = self.y + self.vy * self.dt
        self.z = self.z + self.vz * self.dt

    def UpdateControl(self, vx, vy, vz):
        self.vx = vx
        self.vy = vy
        self.vz = vz

    def UpdateLog(self):
        position = (self.x,self.y,self.z)
        self.traj.append(position)
