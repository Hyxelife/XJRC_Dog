from matplotlib import pyplot as plt
import os
import realtime
import numpy as np


def play_anim():
    shoulderLen =9.88
    armLen = 22.0
    feetLen = 22.2
    toDeg = 180.0/np.pi
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    width = 15+9.41+9.41
    length = 41
    poses = [(width,length),(-width,length),(-width,-length),(width,-length)]

    def drawLeg(topPos,shoulder,arm,feet):
        ax.cla()
        print(f"{shoulder*toDeg},{arm*toDeg},{feet*toDeg}")
        top = np.asarray([topPos[0],topPos[1],0],dtype = np.float32)
        pts = [top]
        curPt = top
        curPt += np.asarray([shoulderLen* np.cos(shoulder),0,shoulderLen*np.sin(shoulder)])
        pts.append(curPt)
        trans = np.asarray([
            [np.cos(shoulder), 0, -np.sin(shoulder)],
            [0, 1, 0],
            [np.sin(shoulder), 0, np.cos(shoulder)]
        ])
        relPt =  np.asarray([0,armLen*np.cos(arm),-armLen*np.sin(arm)])
        pts.append(curPt + trans@relPt)
        relPt += np.asarray([0,-feetLen*np.cos(feet+arm),feetLen*np.sin(feet+arm)])
        pts.append(curPt + trans@relPt)
        pts = np.stack(pts,-1)

        ax.plot(pts[0], pts[1], pts[2])
        #ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2])
        ax.scatter([0], [0], [0],"r")
        ax.set_xlim(-20, 20)
        ax.set_ylim(-20, 20)
        ax.set_zlim(-20, 20)

    angles = []
    with open("../log/record.txt",'r') as f:
        while True:
            line = f.readline()
            if len(line) == 0:
                break
            ang = line.split(",")
            if len(ang) != 4:
                break
            drawLeg(poses[int(ang[0])],float(ang[1]), float(ang[2]), float(ang[3]))
            plt.pause(0.01)

# 按间距中的绿色按钮以运行脚本。
if __name__ == '__main__':
    print("************************")
    print("select mode: \"R\" for realtime, \"S\" for showing log file\n")
    cmd = input("mode:")
    if cmd == 'S':
        play_anim()
    elif cmd == 'R':
        pass