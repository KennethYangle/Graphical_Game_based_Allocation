import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from main import GTA
from collections import deque
# from RflySim.rflyshow import RFly

class Theater:
    def __init__(self, gta):
        self.gta = gta
        self.fig = plt.figure(1)
        self.ax = plt.axes(projection='3d')
        # self.rfly = RFly()

    def view_graph(self):
        for i in range(self.gta.WL.num):
            for j in range(i+1, self.gta.WL.num):
                if self.gta.G[i][j] == 1:
                    self.ax.plot([self.gta.WL.units[i].position[0], self.gta.WL.units[j].position[0]], 
                                 [self.gta.WL.units[i].position[1], self.gta.WL.units[j].position[1]], 
                                 [self.gta.WL.units[i].position[2], self.gta.WL.units[j].position[2]],
                                 color='gray')

    def view_tree(self):
        root = self.gta.algs.tree
        stash = deque()
        stash.append(root)
        while len(stash) != 0:
            r = stash.popleft()
            for c in r.children:
                stash.append(c)
                self.ax.plot([r.val.position[0], c.val.position[0]], 
                             [r.val.position[1], c.val.position[1]], 
                             [r.val.position[2], c.val.position[2]],
                             color='c')


    def view_result(self, results):
        for line in results:
            self.ax.plot([line[0].position[0], line[1].position[0]],
                         [line[0].position[1], line[1].position[1]],
                         [line[0].position[2], line[1].position[2]])


    def render(self):
        for t in self.gta.TL.units:
            self.ax.scatter(t.position[0], t.position[1], t.position[2], marker="*")
            self.ax.text(t.position[0], t.position[1], t.position[2], t.id)
        for w in self.gta.WL.units:
            self.ax.scatter(w.position[0], w.position[1], w.position[2], marker="o")
            self.ax.text(w.position[0], w.position[1], w.position[2], w.id)
        self.ax.scatter(self.gta.WL.center[0], self.gta.WL.center[1], self.gta.WL.center[2], marker="^")
        
        # self.view_graph()
        self.view_tree()
        # self.view_result(self.gta.algs.hungarian_result)
        self.view_result(self.gta.algs.tree_result)
        # self.view_result(self.gta.algs.reallocation)

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        # self.ax.view_init(elev=60,azim=45)
        # self.ax.set_ylim(-50, 100)
        self.ax.grid(False)
        self.ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
        self.ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
        self.ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
        self.fig.savefig("reallocate.png", dpi=600)
        plt.show()
        # self.rfly.render(self.gta.WL, 
        #                  self.gta.TL, 
        #                  np.arctan2(self.gta.algs.direction[1], 
        #                  self.gta.algs.direction[0]), 
        #                  self.gta.algs.tree_result,
        #                  self.gta.algs.tree,
        #                  self.gta.algs.direction)
