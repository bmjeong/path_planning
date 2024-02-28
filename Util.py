import numpy as np
import random
import matplotlib.pyplot as plt
import numpy.linalg as LA

class Obstacle:
    def __init__(self, arr):
        self.n = len(arr) # 꼭지점 개수
        tp = np.transpose(np.asarray(arr))
        self.x = tp[0]
        self.y = tp[1]
        self.center = [sum(self.x) / self.n, sum(self.y) / self.n]

def chk_cross(a,b,p,q): #ab = 직선, pq = 직선
    aa = np.array([a[0], a[1], 0])
    bb = np.array([b[0], b[1], 0])
    pp = np.array([p[0], p[1], 0])
    qq = np.array([q[0], q[1], 0])
    # print(aa,bb,pp,qq)
    c1 = np.cross(bb - aa, pp - aa)
    c2 = np.cross(bb - aa, qq - aa)
    c3 = np.cross(pp - qq, aa - qq)
    c4 = np.cross(pp - qq, bb - qq)
    # print(c1,c2,c3,c4)
    if (np.dot(c1,c2) < 0) & (np.dot(c3,c4) < 0):
        return True
    else:
        return False
def chk_in(obs, center, pt):
    center[0] += random.random()*0.002 - 0.001
    center[1] += random.random() * 0.002 - 0.001
    for i in range(obs.n):
        obs_pt1 = [obs.x[i], obs.y[i]]
        obs_pt2 = [obs.x[(i + 1) % obs.n], obs.y[(i + 1) % obs.n]]
        if chk_cross(center, pt, obs_pt1, obs_pt2):
            return False
    return True

def chk_in_all(obstacles, pt):
    for obs in obstacles:
        if chk_in(obs, obs.center, pt):
            return True
    return False

def chk_collision(obstacles, pt1, pt2):
    for obs in obstacles:
        for i in range(obs.n):
            obs_pt1 = [obs.x[i], obs.y[i]]
            obs_pt2 = [obs.x[(i + 1) % obs.n], obs.y[(i + 1) % obs.n]]
            if chk_cross(pt1, pt2, obs_pt1, obs_pt2):
                return True
    return False

def drawing(vertices, shortest_path, sq, obstacles, G, strr, start, goal):
    x_max = sq[2][0]
    y_max = sq[2][1]
    fig, ax = plt.subplots()
    for i, j in G.edges:
        plt.plot([vertices[int(i)][0], vertices[int(j)][0]], [vertices[int(i)][1], vertices[int(j)][1]])

    length_ = 0
    if shortest_path:
        for i in range(len(shortest_path) - 1):
            XX = [vertices[int(shortest_path[i])][0], vertices[int(shortest_path[i + 1])][0]]
            YY = [vertices[int(shortest_path[i])][1], vertices[int(shortest_path[i + 1])][1]]
            length_ += LA.norm(np.asarray(vertices[int(shortest_path[i])] - vertices[int(shortest_path[i + 1])]))
            plt.plot(XX, YY, 'k')

    for i in range(len(sq)):
        xx = [sq[i][0], sq[(i + 1) % len(sq)][0]]
        yy = [sq[i][1], sq[(i + 1) % len(sq)][1]]
        plt.plot(xx, yy, 'b')

    for obs in obstacles:
        plt.fill(obs.x, obs.y, 'k')

    plt.plot(start[0], start[1], 'ro')
    plt.plot(goal[0], goal[1], 'bo')
    plt.xlim([-5, x_max + 5])
    plt.ylim([-5, y_max + 5])
    plt.title(strr)
    return length_

def cal_len(vertices, shortest_path):
    length_ = 0
    if shortest_path:
        for i in range(len(shortest_path) - 1):
            length_ += LA.norm(np.asarray(vertices[int(shortest_path[i])] - vertices[int(shortest_path[i + 1])]))
    return length_