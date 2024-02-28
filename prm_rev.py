from Util import *
import numpy.linalg as LA
import matplotlib
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi
import networkx as nx
from scipy.spatial import Delaunay
import copy
matplotlib.use("TkAgg")

def gen_pt_from_obs(points, step, Obstacles):
    sq = np.vstack(points)
    for i in range(len(points)):
        p1 = points[i]
        p2 = points[(i + 1) % len(points)]
        ll = LA.norm(p1 - p2)
        if ll > step:
            for j in range(int(ll / step) - 1):
                pt = p1 + (p2 - p1) * (j + 1) / int(ll / step)
                sq = np.vstack((sq, pt))

    for obs in Obstacles:
        for i in range(obs.n):
            p1 = np.asarray([obs.x[i], obs.y[i]])
            p2 = np.asarray([obs.x[(i + 1) % obs.n], obs.y[(i + 1) % obs.n]])
            ll = LA.norm(p1 - p2)
            if ll > step:
                for j in range(int(ll / step)+1):
                    pt = p1 + (p2 - p1) * (j) / int(ll / step)
                    sq = np.vstack((sq, pt))
    return sq

def cal_len(vertices, shortest_path):
    length_ = 0
    if shortest_path:
        for i in range(len(shortest_path) - 1):
            length_ += LA.norm(np.asarray(vertices[int(shortest_path[i])] - vertices[int(shortest_path[i + 1])]))
    return length_
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

def PRM(sq, obstacles, num_node, start, goal):
    ### 초기 점 생성
    vertices = [start, goal]
    cnt2 = 0
    while cnt2 < num_node - 2:
        x1 = random.random()*sq[2][0]
        y1 = random.random()*sq[2][1]
        new_pt = [x1, y1]
        if not chk_in_all(obstacles, new_pt):
            vertices.append(np.asarray(new_pt))
            cnt2 += 1

    shortest_path = []
    Path_Found = False
    while not Path_Found:
        ### 그래프 생성
        G = nx.Graph()
        for i, v in enumerate(vertices):
            G.add_node(str(i), pos=v)

        tri = Delaunay(vertices)
        for i, j, k in tri.simplices:
            if not chk_collision(obstacles, vertices[i], vertices[j]):
                G.add_edge(str(i), str(j), weight=np.linalg.norm(np.array(vertices[i]) - np.array(vertices[j])))
            if not chk_collision(obstacles, vertices[j], vertices[k]):
                G.add_edge(str(j), str(k), weight=np.linalg.norm(np.array(vertices[j]) - np.array(vertices[k])))
            if not chk_collision(obstacles, vertices[k], vertices[i]):
                G.add_edge(str(k), str(i), weight=np.linalg.norm(np.array(vertices[k]) - np.array(vertices[i])))

        try:
            shortest_path = nx.shortest_path(G, source=str(0), target=str(1), weight='weight', method='dijkstra')
        except:
            # print("No path")
            cnt2 = 0
            while cnt2 < 50:
                x1 = random.random() * sq[2][0]
                y1 = random.random() * sq[2][1]
                new_pt = [x1, y1]
                if not chk_in_all(obstacles, new_pt):
                    vertices.append(np.asarray(new_pt))
                    cnt2 += 1
            # print("num", len(vertices))

        if shortest_path:
            Path_Found = True


    return vertices, shortest_path, G

def path_shorten(path, vertices, obstacles):
    new_path = copy.copy(path)
    cnt = 1
    while cnt < len(new_path) - 1:
        if not chk_collision(obstacles, vertices[int(new_path[cnt-1])], vertices[int(new_path[cnt+1])]):
            new_path.remove(new_path[cnt])
        else:
            cnt += 1

    return new_path

if __name__ == '__main__':
    ### 맵 크기 정의
    x_max = 300
    y_max = 300
    sq = np.asarray([[0, 0], [0, y_max], [x_max, y_max], [x_max, 0]])

    ### 시작점, 목표점
    start = [10, 10]
    goal = [290, 290]

    ### Node 개수
    N_node = 100

    ### 장애물 정의 [x1, x2, y1, y2]
    ## Map 1 (simple)
    pol = [[50, 100, 50, 100], [120, 170, 50, 100], [190, 240, 50, 100],
           [50, 100, 120, 170], [120, 170, 120, 170], [190, 240, 120, 170],
           [50, 100, 190, 240], [120, 170, 190, 240], [190, 240, 190, 240]]

    ## Map 2 (simple)
    # pol = [[0, 50, 50, 120], [70, 130, 0, 80], [150, 200, 30, 100], [220, 280, 50, 90],
    #        [130, 170, 120, 200], [190, 250, 110, 150], [200, 300, 180, 200],
    #        [120, 220, 230, 290], [240, 280, 220, 280], [70, 110, 100, 220],
    #        [10, 50, 150, 250]]

    ## Map 3
    # pol = [[50, 60, 0, 100], [50, 60, 110, 300], [60, 110, 50, 61], [90, 100, 60, 280], [170, 270, 200, 210],
    #        [200, 300, 150, 160], [160, 170, 10, 300]]

    ## Map 4
    # pol = [[30, 80, 30, 40], [70, 80, 40, 80], [0, 80, 80, 90], [30, 40, 90, 120],
    #        [120, 130, 0, 120], [80, 170, 120, 130], [70, 80, 120, 200], [0, 40, 150, 160],
    #        [30, 40, 160, 260], [70, 160, 200, 210], [70, 160, 250, 260], [150, 160, 210, 250],
    #        [200, 210, 80, 300], [110, 200, 160, 170], [160, 250, 70, 80], [160, 250, 30, 40],
    #        [240, 250, 40, 70], [250, 300, 120, 130], [250, 260, 130, 170], [200, 280, 200, 210],
    #        [230, 300, 250, 260]]

    ###
    obstacles = []
    for x1, x2, y1, y2 in pol:
        obstacles.append(Obstacle([[x1, y1], [x2, y1], [x2, y2], [x1, y2]]))

    ### 그리도록 하는 플래그
    Draw_flg = True

    ### 장애물만 표시
    if Draw_flg:
        fig, ax = plt.subplots()
        for i in range(len(sq)):
            xx = [sq[i][0], sq[(i + 1) % len(sq)][0]]
            yy = [sq[i][1], sq[(i + 1) % len(sq)][1]]
            plt.plot(xx, yy, 'b')

        for obs in obstacles:
            plt.fill(obs.x, obs.y, 'k')

        plt.plot(start[0], start[1], 'ro')
        plt.plot(goal[0], goal[1], 'bo')

        ax.set_aspect('equal', 'box')
        plt.xlim([-5, x_max + 5])
        plt.ylim([-5, y_max + 5])


    num_node_gvd_prm_set = []
    length_gvd_prm_set = []
    num_node_prm_set = []
    length_prm_set = []

    ### PRM 풀기
    vertices, shortest_path, G = PRM(sq, obstacles, N_node, start, goal)
    num_node_prm = len(vertices)
    shortest_path = path_shorten(shortest_path, vertices, obstacles)
    print("PRM PATH", shortest_path)
    print("Node", num_node_prm)

    ### PRM 그리기
    if Draw_flg:
        length_prm = drawing(vertices, shortest_path, sq, obstacles, G, "PRM", start, goal)
    else:
        length_prm = cal_len(vertices, shortest_path)

    # 그래프를 표시합니다.
    plt.show()
