from helpers import *

class Goal(Potential_Field):
    # This class is related to the current goal
    # This class is intanciated in Sequences sub-classes
    # get()
    # set()
    # get_force()
    list_of_goals = []
    def __init__(self, coord):
        super().__init__()
        self.coord = coord

    def set(self, new_goal):
    # to set the goal
        self.coord = new_goal
        self.list_of_goals.append(new_goal)

    def get(self):
    #    # to use the goal in other classes
        return self.coord 

    def get_force(self, stream):
        p = stream.get_pos()
        g = self.get()
        force = Force()
        if (g[0]-p[0])>0:
            force.x = -self.kd2*(math.exp(-self.kd1*(g[0]-p[0]))-1)
        else:
            force.x = self.kd2*(math.exp(self.kd1*(g[0]-p[0]))-1)
        if (g[1]-p[1])>0:
            force.y = -self.kd2*(math.exp(-self.kd1*(g[1]-p[1]))-1)
        else:
            force.y = self.kd2*(math.exp(self.kd1*(g[1]-p[1]))-1)
        return force.get()
    
    