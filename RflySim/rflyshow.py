# import required libraries
import time
import numpy as np
import os
from collections import deque

# import RflySim APIs
import RflySim.PX4MavCtrlV4 as PX4MavCtrl

class RFly:
    def __init__(self):
        # Create MAVLink control API instance
        self.mav = PX4MavCtrl.PX4MavCtrler(20100)
        # Init MAVLink data receiving loop
        self.mav.InitMavLoop()
        # for vison control, and the second window for observation
        self.mav.sendUE4Cmd(b'RflyChangeMapbyName VisionRingBlank')
        time.sleep(1)
        # Change the target vehicle to copterID=1's vehicle
        self.mav.sendUE4Cmd(b'RflyChangeViewKeyCmd B 1',0)
        time.sleep(0.5)
        # Switch its viewpoint to oboard #1 (front camera view)
        self.mav.sendUE4Cmd(b'RflyChangeViewKeyCmd V 1',0)
        time.sleep(0.5)
        # move the camera to the position [0.3,0,0.05] related to the body
        self.mav.sendUE4Cmd(b'RflyCameraPosAng 0.3 0 0.05 0 0 0',0)
        time.sleep(0.5)

    def draw_line(self, id, model, start, end):
        start = np.array([start.position[0], start.position[1], -start.position[2]])
        end = np.array([end.position[0], end.position[1], -end.position[2]])

        PosE = (start + end) / 2
        AngEuler = [0, 
                    np.arctan2(-end[2] + start[2], np.linalg.norm([end[0] - start[0], end[1] - start[1]])), 
                    np.arctan2(end[1] - start[1], end[0] - start[0])]
        Scale = [np.linalg.norm(start - end), 0.1, 0.1]
        self.mav.sendUE4PosScale(id, model, 0, PosE, AngEuler, Scale)

    def delete(self, id):
        # give an undefined type or infinite far pos
        self.mav.sendUE4Pos(id, 12321, 0, [0,0,0], [0,0,0])

    def render(self, WL, TL, init_yaw, results, tree, direction):
        # view WLs
        wlid_rflyid_mp = dict()     # {WLunit.id: rflysim.id}
        for i in range(len(WL.units)):
            self.mav.sendUE4Pos(i, 3, 0, [WL.units[i].position[0], WL.units[i].position[1], -WL.units[i].position[2]], [0,0,init_yaw])
            wlid_rflyid_mp[WL.units[i].id] = i
        # view TLs
        tlid_rflyid_mp = dict()     # {TLunit.id: rflysim.id}
        for i in range(len(TL.units)):
            self.mav.sendUE4Pos(100+i, 5, 0, [TL.units[i].position[0], TL.units[i].position[1], -TL.units[i].position[2]], [0,0,np.pi+init_yaw])
            tlid_rflyid_mp[TL.units[i].id] = 100+i
        # view results
        for i in range(len(results)):
            self.draw_line(200+i, 207, results[i][0], results[i][1])
        # view tree
        root = tree
        branch_cnt = 0
        stash = deque()
        stash.append(root)
        while len(stash) != 0:
            r = stash.popleft()
            for c in r.children:
                stash.append(c)
                self.draw_line(300+branch_cnt, 207, r.val, c.val)
                branch_cnt += 1

        # delete line
        # time.sleep(30)
        os.system('pause')
        for i in range(len(results)):
            self.delete(200+i)
        for i in range(branch_cnt):
            self.delete(300+i)

        # episode
        K = 1
        v_max = 20
        direction = -direction
        reached_TL = []
        while True:
            # tasks move
            for i in range(len(TL.units)):
                if i not in reached_TL:
                    TL.units[i].position += 0.05 * direction
                    self.mav.sendUE4Pos(100+i, 5, 0, [TL.units[i].position[0], TL.units[i].position[1], -TL.units[i].position[2]], [0,0,np.pi+init_yaw])
            # workers move
            for i in range(len(results)):
                start = results[i][0]
                end = results[i][1]
                if np.linalg.norm(end.position - start.position) < 1:     # reach
                    reached_TL.append(tlid_rflyid_mp[end.id]-100)
                    self.delete(tlid_rflyid_mp[end.id])

                v_cmd = K * (end.position - start.position)
                if (v_norm := np.linalg.norm(v_cmd)) > v_max:
                    v_cmd = v_cmd / v_norm * v_max
                start.position += 0.01 * v_cmd

                if wlid_rflyid_mp[start.id] == 1: continue
                self.mav.sendUE4Pos(wlid_rflyid_mp[start.id], 3, 0, [WL.units[i].position[0], WL.units[i].position[1], -WL.units[i].position[2]], [0,-np.linalg.norm(v_cmd)/100*np.pi/4,init_yaw])
