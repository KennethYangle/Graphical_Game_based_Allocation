import numpy as np
random_seed = 367528110
if random_seed < 0:
    random_seed = np.random.randint(1<<32-1)
np.random.seed(random_seed)
print("random_seed: {}".format(random_seed))

from swarms import Drone, Swarm, Unit, List
from algorithm import Algorithm

import sys
from logger import Logger
sys.stdout = Logger()

class GTA:
    def __init__(self):
        self.IS = Swarm(20, camp="Interceptor", area=[[0,0,220],[100,100,300]], R=50)
        self.HS = Swarm(20, camp="Hostile", area=[[200,200,220],[400,400,300]], R=50)
        print(self.IS)
        print(self.HS)
        self.algs = Algorithm()
        self.TL, self.WL = self.algs.ConstructList(self.HS, self.IS)
        print(self.TL)
        print(self.WL)
        self.G = self.WL.ConstructGraph()
        print(self.G)
        self.algs.GetNeighborhood(self.TL, self.WL)
        self.algs.CalcPayoff(self.TL, self.WL, M=500)
        self.algs.Hungarian(self.TL, self.WL)
        self.algs.SolveGG(self.TL, self.WL)
        # for j in range(1):
        #     self.algs.Reallocation(self.TL, self.WL)

    def main(self):
        from plot import Theater
        theater = Theater(self)
        theater.render()

if __name__ == "__main__":
    gta = GTA()
    gta.main()