import numpy as np
import math
import random
import matplotlib.pyplot as plt
import numpy.linalg as LA

### RRT 용 Node 정의
### parent_index 를 관리한다.
class Node_rrt:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = -1

### Obstacle 용 Node
class Obstacle:
    def __init__(self, arr):
        self.n = len(arr) # 꼭지점 개수
        tp = np.transpose(np.asarray(arr))
        self.x = tp[0]
        self.y = tp[1]
        self.center = [sum(self.x) / self.n, sum(self.y) / self.n]

### RRT Node 확장할 때 필요한 거리, 방향 계산하는 함수
def dist_and_angle(x1,y1,x2,y2):
    dist = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
    angle = math.atan2(y2-y1, x2-x1)
    return(dist,angle)

### 현재 Node 중 가장 가까운 지점 찾는 함수
def nearest_node(x,y):
    temp_dist=[]
    for i in range(len(node_list)):
        dist,_ = dist_and_angle(x,y,node_list[i].x,node_list[i].y)
        temp_dist.append(dist)
    return temp_dist.index(min(temp_dist))

### random point 만드는 함수
def rnd_point(h,l):
    new_y = random.randint(0, h)
    new_x = random.randint(0, l)
    return (new_x,new_y)

### Goal 주변에 도착했는지 알려주는 함수
def chk_goal(goal, x, y):
    if LA.norm([goal[0] - x, goal[1] - y]) < stepSize/2:
        return True
    else:
        return False

### ab, pq 가 서로 교차하는지 알려주는 함수
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

### 장애물의 가장자리와 pt1, pt2 를 이은 직선이 서로 교차하는지 하나하나 확인하는 함수
def chk_collision(obstacles, pt1, pt2):
    for obs in obstacles:
        for i in range(obs.n):
            obs_pt1 = [obs.x[i], obs.y[i]]
            obs_pt2 = [obs.x[(i + 1) % obs.n], obs.y[(i + 1) % obs.n]]
            if chk_cross(pt1, pt2, obs_pt1, obs_pt2):
                return True
    return False



if __name__ == '__main__':
    ### 맵 크기 정의
    Map_x = 300
    Map_y = 300
    ### 초기 노드는 Map 의 중심
    node_list = [Node_rrt(150,150)]
    ### 목적지 정의
    goal = [250, 250]
    ### 한번에 뻗는 가지의 최대 길이
    stepSize = 30

    i = 1
    ### 목적지 도달하면 거기까지 가는 Vertex의 Index 를 저장하는 벡터
    Path = []

    ### 장애물 정의
    obstacles = []
    tp_obs = []
    tp_obs.append([160, 10])
    tp_obs.append([170, 10])
    tp_obs.append([170, 200])
    tp_obs.append([160, 200])
    obstacles.append(Obstacle(tp_obs))


    pathFound = False
    while pathFound == False:
        ### 랜덤 포인드 생성
        nx, ny = rnd_point(Map_x, Map_y)
        print("Random points:", nx, ny, i)

        ### 현재 노드 중 가장 가까운 지점 찾기
        nearest_ind = nearest_node(nx, ny)
        nearest_x = node_list[nearest_ind].x
        nearest_y = node_list[nearest_ind].y

        ### 가지를 뻗는데, 거리가 Stepsize 이하면, 바로 뻗고, 아니면 Stepsize 만큼 뻗음
        dis, theta = dist_and_angle(nearest_x, nearest_y, nx, ny)
        if dis < stepSize:
            tx = nx
            ty = ny
        else:
            tx = nearest_x + stepSize * np.cos(theta)
            ty = nearest_y + stepSize * np.sin(theta)

        ### 뻗은 가지가 장애물과 충돌이 일어나지 않는다면
        if not chk_collision(obstacles, [nearest_x, nearest_y], [tx, ty]):
            ### Node 리스트에 새로운 Vertex를 추가한다
            node_list.append(Node_rrt(tx, ty))
            node_list[i].parent = nearest_ind
            i += 1

            ### 너무 많이 탐색하면 중단
            if i > 1000:
                pathFound = True

            ### GOAL 에 도달하면 중단
            if chk_goal(goal, tx, ty):
                pathFound = True

                ### 반복적으로 Parent를 찾아서 시작점 - Goal 간 경로 연결
                finished = False
                while nearest_ind != -1:
                    Path.append(nearest_ind)
                    nearest_ind = node_list[nearest_ind].parent

    ### 그려진 전체 가지를 그림
    for nn in node_list:
        if nn.parent > -1:
            plt.plot([nn.x, node_list[nn.parent].x], [nn.y, node_list[nn.parent].y], 'k')

    ### 장애물 그리기
    for obs in obstacles:
        plt.fill(obs.x, obs.y)

    ### 목적지까지 가는 경로만 빨강으로 다시 그림
    if Path:
        for j in range(len(Path) - 1):
            xx = [node_list[Path[j]].x, node_list[Path[j + 1]].x]
            yy = [node_list[Path[j]].y, node_list[Path[j + 1]].y]
            plt.plot(xx, yy, 'r')


    plt.show()
