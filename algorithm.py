import numpy as np
from numpy.lib.function_base import _parse_input_dimensions
from swarms import Drone, Swarm, Unit, List
from itertools import permutations
from collections import deque

class Algorithm:
    def __init__(self):
        pass

    def ConstructList(self, H, I):
        len_I = len(I.drones)
        len_H = len(H.drones)
        T = List(H, "Task")
        W = List(I, "Worker")
        W.num = T.num = max(len_I, len_H)

        if len_I == len_H:
            # T.units = H.drones
            for d in H.drones:
                T.units.append(Unit(d, d.id, "Task"))
                T.id_map[d.id] = T.units[-1]
            # W.units = I.drones
            for d in I.drones:
                W.units.append(Unit(d, d.id, "Worker"))
                W.id_map[d.id] = W.units[-1]
        elif len_I < len_H:
            t = (len_H - 1) // len_I + 1
            # T.units = H.drones
            for d in H.drones:
                T.units.append(Unit(d, d.id, "Task"))
                T.id_map[d.id] = T.units[-1]
            cnt = 0
            for k in range(t):
                for i in range(len_I):
                    W.units.append(Unit(I.drones[i], cnt, "Worker"))
                    W.id_map[cnt] = W.units[-1]
                    cnt += 1
                    if cnt >= len_H:
                        break
                if cnt >= len_H:
                        break
        else:
            t = (len_I - 1) // len_H + 1
            cnt = 0
            for k in range(t):
                for i in range(len_H):
                    T.units.append(Unit(H.drones[i], cnt, "Task"))
                    T.id_map[cnt] = T.units[-1]
                    cnt += 1
                    if cnt >= len_I:
                        break
                if cnt >= len_I:
                        break
            # W.units = I.drones
            for d in I.drones:
                W.units.append(Unit(d, d.id, "Worker"))
                W.id_map[d.id] = W.units[-1]

        return T, W

    def GetNeighborhood(self, T, W):
        pos_sum = np.array([0., 0., 0.])
        for i in range(W.num):
            pos_sum += W.units[i].position
        W.center = pos_sum / W.num

        pos_sum = np.array([0., 0., 0.])
        for i in range(T.num):
            pos_sum += T.units[i].position
        T.center = pos_sum / T.num

        # calc NWLs in uniti
        for i in range(W.num):
            for j in range(W.num):
                if W.G[i][j] == 1:
                    W.units[i].NWL.append({"id":j})
            print("NeighborWL {}: {}".format(i, W.units[i].NWL))

        # calc NTLs in uniti
        self.direction = T.center - W.center
        self.direction /= np.linalg.norm(self.direction)
        print("direction: {}".format(self.direction))
        for i in range(W.num):
            for j in range(T.num):
                di = T.units[j].position - W.units[i].position
                # if di.dot(self.direction) / np.linalg.norm(di) > 0.9975:   # cos theta
                #     W.units[i].NTL_central.append({"id":j})
                # if di.dot(self.direction) / np.linalg.norm(di) > 0.985:   # cos theta
                #     W.units[i].NTL.append({"id":j})
                W.units[i].NTL_central.append({"id":j})
                W.units[i].NTL.append({"id":j})
            print("NeighborTL_central {}: {}".format(i, W.units[i].NTL_central))
            print("NeighborTL {}: {}".format(i, W.units[i].NTL))

    def CalcPayoff(self, T, W, M):
        self.M = M
        for i in range(W.num):
            for t in W.units[i].NTL:
                t["payoff"] = self.M - np.linalg.norm(W.units[i].position - T.units[t["id"]].position)
            print("NeighborTL {}: {}".format(i, W.units[i].NTL))

    def Hungarian(self, T, W):
        self.hungarian_result = list()
        from scipy.optimize import linear_sum_assignment
        payoffmat = np.zeros((T.num, W.num))
        print("shape:", payoffmat.shape)
        for i in range(W.num):
            for j in range(T.num):
                payoffmat[i][j] = self.M - np.linalg.norm(W.units[i].position - T.units[j].position)
        cost = -payoffmat
        row_ind, col_ind = linear_sum_assignment(cost)
        # print("Hungarian result: {}".format(col_ind))

        for i in range(len(col_ind)):
            self.hungarian_result.append([T.units[row_ind[i]], W.units[col_ind[i]]])

        GlobalPayoff = -cost[row_ind, col_ind].sum()
        print("GlobalPayoff: {}".format(GlobalPayoff))
        print()

    def CalcCE(self, u: Unit, W):
        per = permutations([i for i in range(len(u.NTL_central))], len(u.NWL))  # p[i] means u.NWL[i]["id"] choose u.NTL_central[p[i]]
        maxv = -1e10
        maxp = None
        for p in per:
            ulocal = 0
            for i, v in enumerate(p):
                # find payoff
                # print(W.units[u.NWL[i]["id"]].NTL)
                for tl in W.units[u.NWL[i]["id"]].NTL:
                    if tl["id"] == u.NTL_central[v]["id"]:
                        ulocal += tl["payoff"]
                        break
            # print("unit {}: p: {}, ulocal: {}".format(u.id, p, ulocal))
            if ulocal > maxv:
                maxv = ulocal
                maxp = p

        for i in range(len(u.NWL)):
            if u.NWL[i]["id"] == u.id:
                idx = i

        if maxp is not None:
            print("unit {} choose task {}".format(u.id, maxp[idx]))

            return maxp[idx]

    def CalcCETree(self, r, W):
        nodes = [r]
        for c in r.children:
            nodes.append(c)
        tasks = [a["id"] for a in r.val.NTL_central]
        print("before", r.val.id, tasks)
        print("parent_select", r.parent_select)
        for sel in r.parent_select:
            if sel in tasks:
                tasks.remove(sel)
        print("after", r.val.id, tasks)
        # extend field of view
        # print("len_tasks: {}, len_nodes: {}".format(len(tasks), len(nodes)))
        if len(tasks) < len(nodes):
            tasks = [a["id"] for a in r.val.NTL]
            for sel in r.parent_select:
                if sel in tasks:
                    tasks.remove(sel)
            print("after-after", r.val.id, tasks)

        per = permutations([i for i in range(len(tasks))], len(nodes))  # p[i] means u.NWL[i]["id"] choose u.NTL_central[p[i]]
        maxv = -1e10
        maxp = None
        for p in per:
            ulocal = 0
            for i, v in enumerate(p):
                # find payoff
                # print(W.units[u.NWL[i]["id"]].NTL)
                is_found = False
                for w in W.units:
                    if w.id == nodes[i].val.id:
                        for tl in w.NTL:
                            if tl["id"] == tasks[v]:
                                ulocal += tl["payoff"]
                                is_found = True
                                break
                        if is_found:
                            break
            # print("unit {}: p: {}, ulocal: {}".format(u.id, p, ulocal))
            if ulocal > maxv:
                maxv = ulocal
                maxp = p

        if maxp is not None:
            print("unit {} choose task {}".format(r.val.id, tasks[maxp[0]]))
            return True, tasks[maxp[0]]
        else:
            maxc, maxi = 0, 0
            for i, a in enumerate(r.val.NTL):
                # print(a)
                if a["payoff"] > maxc:
                    maxc = a["payoff"]
                    maxi = i
            return False, r.val.NTL[maxi]["id"]


    def SolveGG(self, T, W):
        for i in range(W.num):
            W.units[i].cohesion = np.linalg.norm(W.units[i].position - W.center)
        W.units.sort(key=lambda x: (x.cohesion))
        print(W)

        # # Direct allocation
        # self.direct_result = list()
        # print("<-- direct allocation -->")
        # for i in range(W.num):
        #     u_select = self.CalcCE(W.units[i], W)
        #     if u_select is not None:
        #         self.direct_result.append([W.units[i], T.units[u_select]])


        # Tree Graphical allocation
        self.max_tree_width = 4
        self.tree_result = list()
        root = TreeNode(W.units[0])
        open_table = deque()
        open_table.append(root)
        close_table = []
        while len(open_table) != 0:
            r = open_table.popleft()
            r.children = list()
            close_table.append(r)
            cnt = 0
            for i in range(W.num):
                tmpid = W.units[i].id
                if W.G[r.val.id][tmpid] == 1 and tmpid not in [a.val.id for a in close_table] and tmpid not in [a.val.id for a in open_table]:
                    c = TreeNode(W.units[i], children=list())
                    r.children.append(c)
                    open_table.append(c)
                    cnt += 1
                    if cnt >= self.max_tree_width: break
                    print("node {} got unit {}".format(r.val.id, tmpid))
            print(r.val.id, [c.val.id for c in r.children])

        # View Tree Struct
        self.tree = root
        stash = deque()
        stash.append(root)
        while len(stash) != 0:
            r = stash.popleft()
            print("node {}:".format(r.val.id), end=" ")
            for c in r.children:
                stash.append(c)
                print(c.val.id, end=", ")
            print()

        # Solve Tree
        Coverage = 0
        GlobalPayoff = 0

        stash = deque()
        stash.append(root)
        root.parent_select = []
        previous_select = []
        while len(stash) != 0:
            r = stash.popleft()
            # # global share
            # r.parent_select = previous_select
            is_success, r_select = self.CalcCETree(r, W)
            if r_select is not None:
                if is_success:
                    Coverage += 1
                    GlobalPayoff += self.M - np.linalg.norm(r.val.position - T.units[r_select].position)
                    previous_select.append(r_select)
                self.tree_result.append([r.val, T.units[r_select]])
            for c in r.children:
                # # parent share
                # c.parent_select = r.parent_select
                # c.parent_select.append(r_select)

                # local share
                c.parent_select = previous_select           # reference
                # c.parent_select = previous_select.copy()    # shallow copy
                stash.append(c)
        print()
        print("Coverage: {}\nGlobalPayoff: {}".format(Coverage, GlobalPayoff))

    def Reallocation(self, T, W):
        self.reallocation = [[T.units[40], T.units[44]],
                             [T.units[2],  T.units[3]],
                             [T.units[0],  T.units[38]],
                             [T.units[14], T.units[46]],]

class TreeNode:
    def __init__(self, val=None, children=list()):
        self.val = val
        self.children = children