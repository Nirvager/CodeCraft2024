import sys
import random
import numpy as np
from queue import PriorityQueue
from collections import deque
import threading

###########################################################坐标轴统一 x轴向下 y轴向右
# 定义全局变量
n = 200  # 地图的尺寸或某个特定的数量标志
robot_num = 10  # 机器人的数量
berth_num = 10  # 泊位的数量
N = 210  # 用于创建二维数组时的尺寸，略大于n，可能是为了处理边界情况
berth_robotnum = {}   #浅浅记录一下每个港口的机器人数目
special_points = []  #10个港口的位置
special_points_mapp=[] #10个港口在mapp上的位置 注意 mapp是行序优先 与逻辑相反
mapp = np.zeros((210, 210), dtype=int) #二维数组 用于存储地图
path={}   #用来记录总的
dijiecnt = 0  # 用来标记取货物时每轮迪杰斯特拉的上限 以免跳帧跳帧

class Berth:
    # 初始化泊位的属性：位置、运输时间、装载速度
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0):
        self.x = x
        self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed
# 创建泊位实例列表
berth = [Berth() for _ in range(berth_num)]
#ljr 3_7增加了一个存储货物信息的列表
goods=[]
def read_mapp(i,line):
    for j in range(0,n):
        if line[j] == '*' or line[j] == '#':   #障碍物标记为0
            mapp[i][j]=0
        else:
            mapp[i][j]=1          #道路标记为1
#备用替换
class PriorityQueue2:
    def __init__(self):
        self.queue=[]

    def put(self,destination,distance):
        self.queue.append((distance,destination))
        self.queue.sort(key=lambda  x:x[0])

    def get(self):
        return self.queue.pop(0)[1]

    def empty(self):
        return len(self.queue)==0
#用于洗地图的两个函数
def bfs_for_special_points(map_array, start):
    """广度优先搜索，用于找到一个特殊点的连通分支"""
    rows, cols = map_array.shape
    visited = np.zeros_like(map_array, dtype=bool)
    queue = deque([start])
    visited[start] = True

    while queue:
        x, y = queue.popleft()
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols and not visited[nx, ny] and map_array[nx, ny] == 1:
                visited[nx, ny] = True
                queue.append((nx, ny))
    return visited
def retain_connected_special_points(map_array, special_points):
    """保留每组连通的特殊点及其连通分支，其他区域转换为障碍物"""
    all_visited = np.zeros_like(map_array, dtype=bool)

    for sp in special_points:
        if not all_visited[sp]:
            visited = bfs_for_special_points(map_array, sp)
            all_visited |= visited  # 合并这次探索的访问数组到总访问数组

    # 转换未访问的区域为障碍物
    map_array[~all_visited] = 0
    return map_array


#ljr 3.9初步尝试将A*和Dijskra结合
def reconstruct_path2(predecessors, start, goal):
    """
    根据前驱节点字典重建从起点到目标点的路径，并计算路径长度。

    参数:
    predecessors: 前驱节点字典。
    start: 起点坐标(tuple)。
    goal: 目标点坐标(tuple)。

    返回:
    tuple: (路径(list of tuples), 路径长度(int))。
    """
    # if start == goal:
    #     return [start], 0  # 起点即目标点，路径长度为0
    # if goal not in predecessors:
    #     return [], float('inf')  # 目标点不可达

    path = [goal]
    current = goal
    while current in predecessors:
        current = predecessors[current]
        path.append(current)
        if current == start:
            break
    # path.reverse()   这里应该不用倒转了
    # path_length = len(path) - 1
    return path
#目标点是多元的迪杰斯特拉
def dijkstra_to_multiple_goals_with_path_and_length(grid, start, goals):
    """
    使用Dijkstra算法找到从起点到多个目标点的最短路径及其长度。

    参数:
    grid: 二维数组表示的地图，1表示可通行，0表示墙壁。
    start: 起点坐标(tuple)。
    goals: 多个目标点的列表(list of tuples)。

    返回:
    dict: 键为目标点坐标，值为(路径, 路径长度)。
    """
    rows, cols = grid.shape
    distance = np.full((rows, cols), np.inf)
    distance[start] = 0
    visited = np.zeros_like(grid, dtype=bool)
    pq = PriorityQueue()
    pq.put((0, start))
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    predecessors = {}  # 记录前驱节点

    goals_set = set(goals)
    found_goals = set()

    while not pq.empty() and len(found_goals) < len(goals_set):
        current_distance, current_position = pq.get()
        visited[current_position] = True

        if current_position in goals_set:
            found_goals.add(current_position)
            if len(found_goals) == len(goals_set):
                break

        for direction in directions:
            row_offset, col_offset = direction
            neighbor = (current_position[0] + row_offset, current_position[1] + col_offset)

            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and not visited[neighbor]:
                if grid[neighbor] == 1:
                    new_distance = current_distance + 1
                    if new_distance < distance[neighbor]:
                        distance[neighbor] = new_distance
                        predecessors[neighbor] = current_position
                        pq.put((new_distance, neighbor))

    # 重建所有找到的目标点的路径并计算它们的长度
    paths = {goal: reconstruct_path2(predecessors, start, goal) for goal in found_goals}
    return paths



def create_route_memory():
    goals = [tuple(point) for point in np.argwhere(mapp == 1) if tuple(point) not in special_points]
    path[(2,112)] = dijkstra_to_multiple_goals_with_path_and_length(mapp,(2,112), goals)
    path[(2, 187)] = dijkstra_to_multiple_goals_with_path_and_length(mapp, (2, 187), goals)
    path[(31, 61)] = dijkstra_to_multiple_goals_with_path_and_length(mapp, (31, 61), goals)
    path[(116,2)] = dijkstra_to_multiple_goals_with_path_and_length(mapp, (116,2), goals)
    path[(171, 2)] = dijkstra_to_multiple_goals_with_path_and_length(mapp,(171, 2), goals)
    # sys.stderr.write(str(i)+"现在是"+str(i+1)+"\n")

    #path[(berthx,berthy)][(robotx,roboty)]即得到路径
    #字典 套 字典 套 列表
    #港口    机器人位置

def reconstruct_path(predecessors, start, goal):
    """
    根据前驱节点字典重建从起点到目标点的路径，并计算路径长度。

    参数:
    predecessors: 前驱节点字典。
    start: 起点坐标(tuple)。
    goal: 目标点坐标(tuple)。

    返回:
    tuple: (路径(list of tuples), 路径长度(int))。
    """
    # if start == goal:
    #     return [start], 0  # 起点即目标点，路径长度为0
    # if goal not in predecessors:
    #     return [], float('inf')  # 目标点不可达

    path = [goal]
    current = goal
    while current in predecessors:
        current = predecessors[current]
        path.append(current)
        if current == start:
            break
    path.reverse()
    # path_length = len(path) - 1
    return path
#目标点是单元的迪杰斯特拉
def dijkstra(grid, start, goal):
    """
    使用Dijkstra算法找到从起点到多个目标点的最短路径及其长度。

    参数:
    grid: 二维数组表示的地图，1表示可通行，0表示墙壁。
    start: 起点坐标(tuple)。
    goal：目标点

    返回:
    dict: 键为目标点坐标，值为(路径, 路径长度)。
    """
    rows , cols = grid.shape
    distance = np.full((rows, cols), np.inf)
    distance[start] = 0

    visited = np.zeros_like(grid, dtype=bool)

    pq = PriorityQueue()
    pq.put((0, start))
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    predecessors = {}  # 记录前驱节点

    while not pq.empty():
        current_distance, current_position = pq.get()
        visited[current_position] = True

        if current_position == goal:
            break

        for direction in directions:
            row_offset, col_offset = direction
            neighbor = (current_position[0] + row_offset, current_position[1] + col_offset)

            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and not visited[neighbor]:
                if grid[neighbor] == 1:
                    new_distance = current_distance + 1
                    if new_distance < distance[neighbor]:
                        distance[neighbor] = new_distance
                        predecessors[neighbor] = current_position
                        pq.put((new_distance, neighbor))

    # 重建所有找到的目标点的路径并计算它们的长度
    return reconstruct_path(predecessors,start,goal)

#ljr3_8 尝试追踪一下每个点的运动

#计算相反方向 备用
def opposite_direction(direction):
    opposites = {2:3,3:2,1:0,0:1}
    return opposites.get(direction)

#ljr定义一下曼哈顿距离 后续可修改
def distance(a, b) :
    return abs(a[0]-b[0])+abs(a[1]-b[1])


#找最近的港口
def find_nearestberth(robot_pos, berth):
    # near = sorted(berth,key=lambda berth_pos:distance(robot_pos,(berth_pos.x,berth_pos.y)))
    # nearest_berth = near[0]
    # for berth_pos in near:
    #     if berth_robotnum.get((berth_pos.x, berth_pos.y)) <= 3:
    #         nearest_berth = berth_pos
    #         break
    # berth_robotnum[(nearest_berth.x,nearest_berth.y)] +=1
    nearest_berth = min(berth, key=lambda berth_pos: distance(robot_pos, (berth_pos.x, berth_pos.y)))
    return nearest_berth

#避免陷入死角
def is_good(next_pos,dx,dy):
    if mapp[next_pos[0]+dx][next_pos[1]+dy] ==1 or mapp[next_pos[0]-dx][next_pos[1]+dy] == 1 or mapp[next_pos[0]+dx][next_pos[1]-dy] == 1:
        return True
    return False

#使用一个简单的A*启发式搜索
def get_feasible_directions(current_pos,move_history):
    """获取没有被障碍物阻塞的方向"""
    directions = [2, 3, 1, 0] #分别对应 上下左右  #图中左上角为原点 向下为x正 向右为y正
    dx_dy=[(-1,0),(1,0),(0,-1),(0,1)]
    feasible_directions = []

    #zip是一个合成元组的东西
    for direction,(dx,dy) in zip(directions,dx_dy):
        next_pos = (current_pos[0]+dx,current_pos[1]+dy)
        #检查下一步是否被障碍物阻塞 can be optimized in the future
        if mapp[next_pos[0]][next_pos[1]]==1 and not is_repetitive_movement(move_history,(current_pos,direction)) and is_good(next_pos,dx,dy) and not is_robot_here(next_pos[0],next_pos[1]):
            feasible_directions.append((direction,next_pos))

    return feasible_directions


def is_repetitive_movement(move_history, choice):
    """检查重复模式"""
    if len(move_history) <=3:
        return False
    if choice[1] == opposite_direction(move_history[-1][1]):
        return True
    return False

def is_circulate_movement(move_history,choice):
    if choice in move_history:
        return True
    return False


def is_robot_here(x,y):
    for i in range (robot_num):
        if x == robot[i].x and y == robot[i].y:
            return True
    return False

#这个要修改 启发式函数
def heuristic(next_pos,goal_pos):
    base_cost = distance(next_pos,goal_pos)
    # 如果已经很接近目标，可能不需要调整成本
    if base_cost < 100:
        return base_cost

    obstacle_cost = 0
    robot_cost = 1
    dx = [1, -1, 0, 0]
    dy = [0, 0, 1, -1]
    for i in range(4):
        new_x, new_y = next_pos[0] + dx[i], next_pos[1] + dy[i]
        # 确保不会检查地图边界之外的位置
        if 0 <= new_x < len(ch) and 0 <= new_y < len(ch[0]) and ch[new_x][new_y] != '.':
            obstacle_cost += 5  # 对于每个障碍物增加额外的成本
        if is_robot_here(new_x,new_y):
            robot_cost = robot_cost*4

    #动态调整障碍物成本，距离目标越远，障碍物影响越小
    adjusted_obstacle_cost = obstacle_cost * ( 5/ (base_cost + 1))

    return base_cost + adjusted_obstacle_cost + robot_cost

def get_best_direction(current_pos,goal_pos,move_history):
    """基于当前位置、目标位置和障碍物，决定最优的单步移动方向"""
    feasible_directions = get_feasible_directions(current_pos,move_history)

    #计算每个可行方向的启发式成本
    best_direction = -1
    min_cost = float('inf')
    for direction,next_pos in feasible_directions:
        #启发式成本必改
        # cost = heuristic(next_pos,goal_pos)
        cost = distance(next_pos,goal_pos)
        # if cost < min_cost and not is_circulate_movement(move_history,(current_pos,direction)):
        if cost < min_cost and not is_circulate_movement(move_history,(current_pos,direction)):
            min_cost=cost
            best_direction=direction
        #为了解决墙的问题做出的尝试
        elif cost == min_cost and len(move_history)>=2:
            if direction == move_history[-1][1]:
                best_direction = direction

    #如果被限制了 则wander一下
    if best_direction not in range(0,4):
        best_direction = wander(current_pos[0],current_pos[1])
    return best_direction

#将路径转换为方向
def jiexi_path(path):
    list=[]
    for i in range(len(path)-1):
        dx = path[i+1][0] - path[i][0]
        dy = path[i+1][1] - path[i][1]
        if (dx,dy) == (1,0):
            list.append(3)
        elif (dx,dy) == (-1,0):
            list.append(2)
        elif (dx,dy) == (0,1):
            list.append(0)
        else:
            list.append(1)
    return list

def find_direction_according_to_route(route,index):
    # sys.stderr.write(str(index) + " " + str(index+1)+" "+str(len(route)))
    # sys.stderr.write(str(route[index])+" "+str(route[index+1]))
    dx = route[index+1][0]-route[index][0]
    dy = route[index+1][1]-route[index][1]

    if (dx, dy) == (1, 0):
        return 3
    elif (dx, dy) == (-1, 0):
        return 2
    elif (dx, dy) == (0, 1):
        return 0
    else:
        return 1


def A_star(robot_pos,target,history):
    direction = get_best_direction(robot_pos,target,history)
    return direction


def wander(x,y):
    if mapp[x+1][y] == 1:
        return 3
    if mapp[x-1][y] == 1:
        return 2
    if mapp[x][y+1] == 1:
        return 0
    if mapp[x][y-1] == 1:
        return 1

#第一次就找小范围内的 没有就第二次寻找   假设distance15能有效由A*解决
def find_nearestgoods(robot_pos, goods):
    nearest_goods = None
    for good in goods:
        if distance(good[0],robot_pos) <= 8:
            nearest_goods = good
            break
    # nearest_goods = min(goods,key=lambda good_pos:distance(robot_pos,good_pos[0]))
    return nearest_goods
#第二次找
def  find_secondnearestgoods(robot_pos,goods):
    nearest_goods = min(goods,key=lambda good_pos:distance(robot_pos,good_pos[0]))
    # if len(goods) >=5:
    #     goods.remove(nearest_goods)
    return nearest_goods

def check_is_good_alive(good_pos):
    for good in goods:
        if good[0]==good_pos:
            return True
    return False


berth_tmp=[(2,112),(2,112),(2, 187),(2, 187),(31, 61),(31, 61),(116,2),(116,2),(171, 2),(171, 2)]
crash_accident=[]
# 类定义
class Robot:
    # 初始化机器人的属性：位置、携带的货物、状态、目标泊位的坐标
    def __init__(self, startX=0, startY=0, goods=0, status=0, mbx=0, mby=0, mbgx=0,mbgy=0):
        self.x = startX
        self.y = startY
        self.goods = goods
        self.status = status
        self.mbx = mbx  #港口目标
        self.mby = mby
        #针对目标货物的    都是用于迪杰斯特拉的
        self.mbgx =mbgx #
        self.mbgy = mbgy
        self.mbg_zhen = 0
        self.mbg_val=0
        #港口迪杰斯特拉路线
        self.route=[]
        self.route_berth_index = 0
        #货物迪杰斯特拉路线
        self.goods_route=[]
        self.route_goods_index = 0

        self.flag = 0   #记录转换点
        self.move_history = []   #用于A*算法
        self.is_alive = 1   #这个机器人是活的
        self.is_crash = 0  #用来记录是否发生了碰撞   0表示正常 1表示撞车

    def change(self,direction):
        list = []
        if direction == 3 or direction==2:
            if mapp[self.x][self.y+1]==1:
                list.append(0)
            if mapp[self.x][self.y -1] == 1:
                list.append(1)
            if len(mapp):
                list.append(-2)
                return random.choice(list)
            else:
                return -2
        if direction == 0 or direction==1:
            if mapp[self.x+1][self.y]==1:
                list.append(3)
            if mapp[self.x-1][self.y] == 1:
                list.append(2)
            if len(mapp):
                list.append(-2)
                return random.choice(list)
            else:
                return -2

    def change_after_crash(self,flag):
    #卧槽了
        list = []
        #处理左下方 我们可以选择 右上
        if flag == 1 or flag == 3 or flag == 7:
            if mapp[self.x][self.y+1]==1:
                list.append(0)
            if mapp[self.x-1][self.y]==1:
                list.append(2)
            if len(list):
                list.append(-2)
                direction = random.choice(list)
            else:
                direction = -2
        #处理右上方 我们可以选择左下
        if flag == 4 or flag == 6 or flag == 2:
            if mapp[self.x][self.y-1]==1:
                list.append(1)
            if mapp[self.x+1][self.y]==1:
                list.append(3)
            if len(list):
                list.append(-2)
                direction = random.choice(list)
            else:
                direction = -2
        #左上 可以选择右下
        if flag == 5:
            if mapp[self.x][self.y+1]==1:
                list.append(0)
            if mapp[self.x+1][self.y]==1:
                list.append(3)
            if len(list):
                list.append(-2)
                direction = random.choice(list)
            else:
                direction = -2
        #右下 可选择左上
        if flag == 8:
            if mapp[self.x][self.y-1]==1:
                list.append(1)
            if mapp[self.x-1][self.y]==1:
                list.append(2)
            if len(list):
                list.append(-2)
                direction = random.choice(list)
            else:
                direction = -2

        # if flag == 11 or flag == 13 or flag == 17:
        #     if mapp[self.x][self.y + 1] == 1:
        #         list.append(0)
        #     if mapp[self.x - 1][self.y] == 1:
        #         list.append(2)
        #     if len(list):
        #         list.append(-2)
        #         direction = random.choice(list)
        #     else:
        #         direction = -2
        # if flag ==14 or flag == 16 or flag == 12:
        # if flag == 5:
        # if flag == 8:
        return direction
    def check_if_there_isa_robot_around(self,x,y):
        dx = [-1,0,1]
        dy = [-1,0,1]
        dx2=[-2,0,2]
        dy2=[-2,0,2]
        flag=0
        for i in range(3):
            for j in range(3):
                if is_robot_here(x+dx[i],y+dy[j]):
                    #左边
                    if dx[i] == 0 and dy[j]==-1:
                        flag = 1
                    #右边
                    if dx[i] == 0 and dy[j]==1:
                        flag =2
                    #下方
                    if dx[i] == 1 and dy[j]==0:
                        flag = 3
                    #上方
                    if dx[i] == -1 and dy[j]==0:
                        flag = 4
                    #左上方
                    if dx[i] == -1 and dy[j]==-1:
                        flag = 5
                    #右上方
                    if dx[i] == -1 and dy[j]==1:
                        flag =6
                    #左下方
                    if dx[i] == 1 and dy[j]==-1:
                        flag = 7
                    #右下方
                    if dx[i] == 1 and dy[j]==1:
                        flag = 8
        # if flag:
        #     target=(x+dx[i],y+dy[j])
        # for i in range(3):
        #     for j in range(3):
        #         if is_robot_here(x+dx2[i],y+dy2[j]):
        #             #左边
        #             if dx[i] == 0 and dy[j]==-2:
        #                 flag = 11
        #             #右边
        #             if dx[i] == 0 and dy[j]==2:
        #                 flag = 12
        #             #下方
        #             if dx[i] == 2 and dy[j]==0:
        #                 flag = 13
        #             #上方
        #             if dx[i] == -2 and dy[j]==0:
        #                 flag = 14
        #             #左上方
        #             if dx[i] == -2 and dy[j]==-2:
        #                 flag = 15
        #             #右上方
        #             if dx[i] == -2 and dy[j]==2:
        #                 flag = 16
        #             #左下方
        #             if dx[i] == 2 and dy[j]==-2:
        #                 flag = 17
        #             #右下方
        #             if dx[i] == 2 and dy[j]==2:
        #                 flag = 18
        return flag
    def find_available_direction(self,x,y):
        flag=0  #判断是否是因为机器人间碰撞
        #判断一个四周是否有机器人
        flag= self.check_if_there_isa_robot_around(x,y)
        if flag:
            direction = self.change_after_crash(flag)
        #是撞地图的话
        else:
            if self.route:
                direction = self.change(opposite_direction(find_direction_according_to_route(self.route,self.route_berth_index-1)))
            else:
                direction = wander(self.x,self.y)
        return direction
    def crash_decide(self):
        direction = self.find_available_direction(self.x,self.y)
        if self.route:
            self.route=[]
            self.route_berth_index=0
        if self.goods_route:
            self.goods_route=[]
            self.route_goods_index=0
            self.mbgx=0
            self.mbgy=0
        return direction
    def search_goods(self,dijiecnt):
        if self.is_crash == 1:
            direction = self.crash_decide()
            self.is_crash = 0
            return direction
        tmp_good = find_nearestgoods((self.x,self.y), goods)    #这是为了应对 假如迪杰斯特拉很远 在路程中碰见很近的可以转换目标
        if tmp_good != None:
            if distance(tmp_good[0],(self.x,self.y)) < distance((self.x,self.y),(self.mbgx,self.mbgy)):
                # sys.stderr.write("我杀死了dd")
                self.route_goods_index=0
                self.goods_route=[]
                self.mbgx=0
                self.mbgy=0
                direction = get_best_direction((self.x, self.y), (tmp_good[0][0], tmp_good[0][1]),self.move_history)
                self.move_history.append(((self.x, self.y), direction))
                if len(self.move_history) >= 10:
                    self.move_history.pop(0)
                return direction

        #判断一下这个目标货物是不是消失了
        if check_is_good_alive((self.mbgx,self.mbgy)) == False:
            self.route_goods_index = 0
            self.goods_route = []
            self.mbgx = 0
            self.mbgy = 0

        if self.goods_route == [] or self.mbgx==0 and self.mbgy == 0:
            nearest_goods = find_secondnearestgoods((self.x, self.y), goods)
            #太远了我们还是A*一下 减少迪杰斯特拉的时间
            if distance(nearest_goods[0],(self.x, self.y)) >= 25 or dijiecnt >=1:
                #这个地方可以修改
                # better_nearest_goods = half_nearest(nearest_goods)
                direction = get_best_direction((self.x,self.y),nearest_goods[0],self.move_history)
                self.move_history.append(((self.x, self.y), direction))
                if len(self.move_history) >= 20:
                    self.move_history.pop(0)
                return direction
            # 记录一下基本信息
            self.mbgx = nearest_goods[0][0]
            self.mbgy = nearest_goods[0][1]
            self.mbg_zhen = nearest_goods[1]
            self.mbg_val = nearest_goods[2]

            self.goods_route = dijkstra(mapp, (self.x, self.y), (self.mbgx, self.mbgy))
            dijiecnt +=1

            # sys.stderr.write("我找到了缔结斯特勒")
            self.route_goods_index = 0

            # 以防万一index哪里出错了
        if self.route_goods_index > len(self.goods_route) - 2:
            # sys.stderr.write(str(self.route_goods_index)+"我在爆炸"+str(len(self.goods_route)))
            self.goods_route = []
            self.route_goods_index = 0
            direction = A_star((self.x, self.y), (self.mbgx, self.mbgy), self.move_history)
            self.move_history.append(((self.x, self.y), direction))
            if len(self.move_history) >= 20:
                self.move_history.pop(0)
            return direction

        if len(self.goods_route) < 5:
            # sys.stderr.write("我快到了")
            direction = A_star((self.x, self.y), (self.mbgx, self.mbgy), self.move_history)
            self.move_history.append(((self.x, self.y), direction))
            if len(self.move_history) >= 20:
                self.move_history.pop(0)
        else:
            if self.goods_route:
                # sys.stderr.write("我知道怎么走")
                direction = find_direction_according_to_route(self.goods_route, self.route_goods_index)
                self.route_goods_index += 1
            else:  # 可能这一帧没算出来路径
                # sys.stderr.write("我根本不知道怎么走")
                direction = A_star((self.x, self.y), (self.mbgx, self.mbgy), self.move_history)
                self.move_history.append(((self.x, self.y), direction))
                if len(self.move_history) >= 20:
                    self.move_history.pop(0)
        return direction
        # #如果不是走迪杰斯特拉而是A* 或者还没有定迪杰斯特拉
        # if self.mbgx == 0 and self.mbgy ==0:
        #     nearest_goods = find_nearestgoods((self.x,self.y), goods)
        #     if nearest_goods != None:
        #         direction = get_best_direction((self.x,self.y), (nearest_goods[0][0], nearest_goods[0][1]), self.move_history)
        #         self.move_history.append(((self.x,self.y), direction))
        #         if len(self.move_history) >= 15:
        #             self.move_history.pop(0)
        #     else:
                #找不到附近的只能迪杰斯特拉先走
        #         # sys.stderr.write("我在找迪杰斯特拉")
        #         nearest_goods = find_secondnearestgoods((self.x,self.y), goods)
        #         # sys.stderr.write("目标是"+str(nearest_goods[0][0])+str(nearest_goods[0][1])+" ")
        #
        #         self.mbgx = nearest_goods[0][0]
        #         self.mbgy = nearest_goods[0][1]
        #         self.mbg_zhen=nearest_goods[1]
        #         self.mbg_val=nearest_goods[2]
        #
        #         self.goods_route = dijkstra(mapp,(self.x,self.y),(self.mbgx,self.mbgy))
        #         #若道路不为空
        #         if self.goods_route:
        #             sys.stderr.write(str(self.goods_route)+"\n")
        #             direction = find_direction_according_to_route(self.goods_route,0)
        #             self.route_goods_index += 1
        #         else:
        #             direction = get_best_direction((self.x,self.y),(self.mbgx,self.mbgy),self.move_history)
        #             self.move_history.append(((self.x, self.y), direction))
        #             if len(self.move_history) >= 20:
        #                 self.move_history.pop(0)
        # #其余情况就是正在走迪杰斯特拉
        # else:
        #     sys.stderr.write("我的位置是"+str(self.x)+" "+ str(self.y))
        #     if self.route_goods_index <= len(self.goods_route) -2:
        #         direction = find_direction_according_to_route(self.goods_route,self.route_goods_index)
        #         self.route_goods_index+=1
        #     else:
        #         direction = wander(self.x,self.y)

        # sys.stderr.write(str(direction))
        # return direction

    #move2是迪杰斯特拉 港口对全图嵌入后的移动策略
    def move2(self,i,dijiecnt):
        if self.is_crash == 1:
            direction = self.crash_decide()
            self.is_crash = 0
            return direction
        if self.goods == 1:

            #不知道为什么需要这个 打个补丁
            if (self.x,self.y) in special_points:
                direction = wander(self.x,self.y)
                return direction

            if self.route == []:
                # nearest_berth = find_nearestberth((self.x, self.y), berth)
                # nearest_berth = berth[(i//2)*2]
                # if i==5:
                #     nearest_berth = berth[7]
                (nearest_berth_x,nearest_berth_y) = berth_tmp[i]
                self.mbx=nearest_berth_x
                self.mby=nearest_berth_y
                berth_robotnum[(self.mbx,self.mby)] +=1

                # sys.stderr.write("当前的点在"+str(self.x)+" "+str(self.y)+"目标港口为"+str(nearest_berth.x)+str(nearest_berth.y)+"\n")
                self.route = path[(nearest_berth_x, nearest_berth_y)][(self.x, self.y)]
                self.route_berth_index = 0


            if self.route_berth_index > len(self.route) - 2:
                self.route = []
                self.route_berth_index = 0
                direction = A_star((self.x, self.y), (self.mbx, self.mby), self.move_history)
                self.move_history.append(((self.x, self.y), direction))
                if len(self.move_history) >= 10:
                    self.move_history.pop(0)
                return direction


            if len(self.route) <= 15:
                direction = A_star((self.x,self.y),(self.mbx,self.mby),self.move_history)
                self.move_history.append(((self.x, self.y), direction))
                if len(self.move_history) >= 15:
                    self.move_history.pop(0)
                self.route=[]
                self.route_berth_index=0
                # if self.mbx <= self.x and self.x <=self.mbx+3 and self.mby<=self.y and self.y<=self.y+3:
                #     self.route = []
            else:
                direction = find_direction_according_to_route(self.route,self.route_berth_index)
                self.route_berth_index += 1
            return direction

        if self.goods == 0:
            direction = self.search_goods(dijiecnt)
            if direction <0:
                sys.stderr.write(str(self.goods_route)+"这里222222有问题")
            return direction
    #move是dijie未结束时的策略
    def move(self,i,dijiecnt):
        #某个撞车的函数
        #撞车了要回溯  主要是感觉不太会装车吧
        # if self.status == 0:
        #     direction = opposite_direction(self.move_history[-1][1])
        #     self.move_history.append(((self.x,self.y),direction))
        #     if len(self.move_history) >= 125:
        #         self.move_history.pop(0)
        #     return direction
        if self.is_crash == 1:
            direction = self.crash_decide()
            self.is_crash = 0
            return direction

        if self.goods == 1:
            if self.route == []:
                # nearest_berth = find_nearestberth((self.x, self.y), berth)
                # nearest_berth = berth[(i//2)*2]
                # if i==5:
                #     nearest_berth = berth[7]
                (nearest_berth_x,nearest_berth_y) = berth_tmp[i]
                self.mbx=nearest_berth_x
                self.mby=nearest_berth_y
                # if distance((self.x,self.y),(self.mbx,self.mby))>=20:
                self.route=dijkstra(mapp,(self.x,self.y),(self.mbx,self.mby))
                self.route_berth_index=0

            # sys.stderr.write("hhhhhhhh"+str(self.route))
            if self.route_berth_index > len(self.route) - 2:
                self.route = []
                self.route_berth_index = 0
                direction = A_star((self.x, self.y), (self.mbx, self.mby), self.move_history)
                self.move_history.append(((self.x, self.y), direction))
                if len(self.move_history) >= 10:
                    self.move_history.pop(0)
                return direction

            if distance((self.x,self.y),(self.mbx,self.mby))<10:
                direction=A_star((self.x,self.y),(self.mbx,self.mby),self.move_history)
                self.move_history.append(((self.x,self.y),direction))
            else:
                # if self.route_berth_index:  # 检查索引是否在有效范围内
                # if self.route == []:
                if self.route:  # 确保route不为空
                    direction = find_direction_according_to_route(self.route,self.route_berth_index)
                    self.route_berth_index +=1
                else:
                    direction = A_star((self.x,self.y),(self.mbx,self.mby),self.move_history)
                    self.move_history.append(((self.x, self.y), direction))
            if len(self.move_history) >= 10:
                self.move_history.pop(0)
            return direction

        #未安装货物
        if self.goods == 0:
            direction = self.search_goods(dijiecnt)
            if direction <0:
                sys.stderr.write(str(self.goods_route)+"这里有问题")
            return direction



# 创建机器人实例列表
robot = [Robot() for _ in range(robot_num + 10)]


#准备写一个go函数
class Boat:
    # 初始化船的属性：编号、位置、状态
    def __init__(self, num=0, pos=0, status=0):
        self.num = num
        self.pos = pos
        self.status = status

    #go 船装完货离开
    def go(self,berth_id):
        print(1)

    #ship  船选择港口停靠 策略？
    def ship(self,berth_id):
        print(2)


# 创建船实例列表
boat = [Boat() for _ in range(10)]

# 更多全局变量定义
money = 0  # 货币数量
boat_capacity = 0  # 船的容量
id = 0
ch = []  # 可能用于存储地图信息   可以修改感觉
gds = [[0 for _ in range(N)] for _ in range(N)]  # 二维数组，用于存储货物信息

def Init(mapp,t1):

    # 初始化函数，用于从输入读取初始配置
    #地图
    for i in range(0, n):
        line = input()
        read_mapp(i,line)
    #港口信息
    for i in range(berth_num):
        line = input()
        berth_list = [int(c) for c in line.split(sep=" ")]
        id = berth_list[0]
        berth[id].x = berth_list[1]
        berth[id].y = berth_list[2]
        berth_robotnum[(berth_list[1], berth_list[2])] = 0

        #测试只有5个迪杰斯特拉
        if i == 0 or i == 2 or i == 6 or i==7 or i == 8:
            special_points.append((berth[id].x,berth[id].y))

        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]

        sys.stderr.write("港口的位置:"+str(berth[id].x)+" "+str(berth[id].y)+"\n")
    boat_capacity = int(input())

    #对mapp进行预处理
    mapp = retain_connected_special_points(mapp,special_points)
    for rt in robot:
        if mapp[rt.x][rt.y] == 0:
            rt.is_alive = 0

    t1.start()

    okk = input()
    print("OK")
    sys.stdout.flush()

#ljr增加了一个按照时间删除货物的函数
def delete_goods(zhen):
    for good in goods:
        if good[1] - zhen >=1000:
            goods.remove(good)
#去除一些位置不好的货物
def check_goods(x,y):
    if mapp[x][y] == 0:
        return False
    cnt = 0
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        if mapp[x+dx][y+dy] == 0:
            cnt +=1
    if cnt>=2:
        return False
    return True

#按照价值大小插入
# def goods_append(n):
#     for i in range(len(goods)):
#         if good[2] > goods[i][2]+10:
#             goods.insert(i,good)
#             return goods
#     goods.append(good)
#     return goods

def Input(zhen):
    # 输入函数，用于在每一"帧"读取新的输入数据并更新系统状态
    id, money = map(int, input().split(" "))
    num = int(input())
    for i in range(num):
        x, y, val = map(int, input().split())
        #增加一步检测 若货物在死位则不增加
        if check_goods(x,y) == False:
            continue
        else:
            goods.append(((x,y),zhen,val))

    for i in range(robot_num):
        robot[i].goods, robot[i].x, robot[i].y, robot[i].status = map(int, input().split())

    for i in range(5):
        boat[i].status, boat[i].pos = map(int, input().split())

    okk = input()
    return id

#ljr3.8判断一下机器人是否在港口
def is_robot_at_any_port(robot, berth):
    for port in berth:
        if port.x <= robot.x and robot.x <=port.x+3 and port.y<=robot.y and robot.y <=port.y+3:
            return True
    return False

def is_robot_at_any_goods(robot,goods):
    for goods_pos in goods:
        if goods_pos[0][0] == robot.x and goods_pos[0][1] == robot.y:
            return True
    return False

#取完货物删除货物
def goods_go(robot_pos):
    for good in goods:
        if good[0] == robot_pos:
            goods.remove(good)

def check_robot(x,y):
    cnt = 0
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        checkmate = (x+dx,y+dy)
        for rt in robot:
            if (rt.x,rt.y) == checkmate:
                cnt += 1
    if cnt >=2:
        return False
    return True



if __name__ == "__main__":
    # 主程序入口
    t1 = threading.Thread(target=create_route_memory)
    Init(mapp,t1)  # 调用初始化函数
    # 创造一个线程单独来干迪杰斯特拉的活
    sign=1   #用来标记第一次进入t1.is_dead 便于初始化
    sign2=1   #用来标记第一次进入t1.is_alive 便于初始化

    for zhen in range(1, 15001):
        delete_goods(zhen)
        sys.stderr.write(str(t1.is_alive()))
        # 主循环，模拟15000帧的运
        id = Input(zhen)  # 处理每帧的输入
        if zhen == 100:
            print("ship",0,2)
            print("ship",1,7)
            print("ship", 2, 5)
            print("ship", 3, 9)
            print("ship", 4, 0)
        if zhen == 12000:
            print("go",1)
            print("go", 4)
            print("go",2)
            print("go",3)
            print("go", 0)

        if t1.is_alive():  #当迪杰斯特拉整体没有算完的时候
            if sign2 == 1:
                for rt in robot:
                    rt.route=[]
                    rt.route_berth_index=0
            dijiecnt = 0
            for i in range(robot_num):
                if mapp[robot[i].x][robot[i].y] == 0:
                    continue
                #说明撞车了
                if robot[i].status == 0:
                    robot[i].is_crash = 1
                    continue

                if is_robot_at_any_goods(robot[i], goods):
                    sys.stderr.write("找到一个")
                    print("get", i)

                    #找到货物 开始走向码头 进行一系列初始化
                    robot[i].route = []
                    robot[i].route_berth_index = 0
                    robot[i].mbx=0
                    robot[i].mby=0
                    #找到货物后 为下一次找货物做初始化
                    robot[i].mbgx=0
                    robot[i].mbgy=0
                    robot[i].goods_route=[]
                    robot[i].route_goods_index = 0
                    robot[i].move_history=[]

                    goods_go((robot[i].x,robot[i].y))

                if is_robot_at_any_port(robot[i], berth):
                    print("pull", i)

                    # if robot[i].flag == 1:
                    #     berth_robotnum[(robot[i].mbx, robot[i].mby)] -= 1
                    # robot[i].flag = 0

                    #到港口 为下一次到港口进行一系列初始化
                    robot[i].route = []
                    robot[i].route_berth_index = 0
                    robot[i].mbx=0
                    robot[i].mby=0
                    #到港口 开始找货物做初始化
                    robot[i].mbgx=0
                    robot[i].mbgy=0
                    robot[i].goods_route=[]
                    robot[i].route_goods_index = 0
                    robot[i].move_history = []

                # 控制每个机器人移动
                # if check_robot(robot[i].x,robot[i].y) == True:
                direction = robot[i].move(i,dijiecnt)
                if direction in range(0,4):
                    print("move", i, direction)
                sign2=0
                sys.stdout.flush()
        else: #迪杰斯特拉算完之后 到港口直接使用迪杰斯特拉
            #第一次先初始化 去掉之前的痕迹
            if sign == 1:
                for i in range(robot_num):
                    robot[i].route=[]
                    robot[i].route_berth_index=0
            dijiecnt = 0
            for i in range(robot_num):
                if mapp[robot[i].x][robot[i].y] == 0:
                    continue

                #说明撞车了
                if robot[i].status == 0:
                    robot[i].is_crash = 1
                    continue

                if is_robot_at_any_goods(robot[i], goods):
                    sys.stderr.write("找到一个")
                    print("get", i)
                    #找到货物 开始走向码头 进行一系列初始化
                    robot[i].route = []
                    robot[i].route_berth_index = 0
                    robot[i].mbx=0
                    robot[i].mby=0
                    #找到货物后 为下一次找货物做初始化
                    robot[i].mbgx=0
                    robot[i].mbgy=0
                    robot[i].goods_route=[]
                    robot[i].route_goods_index = 0
                    robot[i].move_history = []

                    goods_go((robot[i].x,robot[i].y))

                if is_robot_at_any_port(robot[i], berth):
                    print("pull", i)

                    # 这里考虑到了密度 后面要改
                    # if robot[i].flag == 1:
                    #     berth_robotnum[(robot[i].mbx, robot[i].mby)] -= 1
                    # robot[i].flag = 0
                    #到港口 为下一次到港口进行一系列初始化
                    robot[i].route = []
                    robot[i].route_berth_index = 0
                    robot[i].mbx=0
                    robot[i].mby=0
                    #到港口 开始找货物做初始化
                    robot[i].mbgx=0
                    robot[i].mbgy=0
                    robot[i].goods_route=[]
                    robot[i].route_goods_index = 0
                    robot[i].move_history = []
                    
                # if check_robot(robot[i].x, robot[i].y) == True:
                direction = robot[i].move2(i,dijiecnt)
                if direction in range(0,4):
                    print("move", i, direction)
                sys.stdout.flush()
                sign = 0
        print("OK")
        sys.stdout.flush()
