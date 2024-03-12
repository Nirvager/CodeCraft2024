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
    for i in range(0, 10, 2):
        path[(berth[i].x,berth[i].y)]= dijkstra_to_multiple_goals_with_path_and_length(mapp, special_points[i], goals)
        path[(berth[i+1].x,berth[i+1].y)] = dijkstra_to_multiple_goals_with_path_and_length(mapp, special_points[i + 1], goals)

    sys.stderr.write(str(i)+"现在是"+str(i+1)+"\n")

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

#找最近的货物
def find_nearestgoods(robot_pos, goods):
    nearest_goods = None
    for good in goods:
        if distance(good[0],robot_pos) <= 50:
            nearest_goods = good
            break
    return nearest_goods

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
        if mapp[next_pos[0]][next_pos[1]]==1 and not is_repetitive_movement(move_history,(current_pos,direction)) and is_good(next_pos,dx,dy):
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
    best_direction = random.randint(0,3)
    min_cost = float('inf')
    for direction,next_pos in feasible_directions:
        #启发式成本必改
        # cost = heuristic(next_pos,goal_pos)
        cost = distance(next_pos,goal_pos)
        if cost < min_cost and not is_circulate_movement(move_history,(current_pos,direction)):
            min_cost=cost
            best_direction=direction

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
    else:
        return 1
# 类定义
class Robot:
    # 初始化机器人的属性：位置、携带的货物、状态、目标泊位的坐标
    def __init__(self, startX=0, startY=0, goods=0, status=0, mbx=0, mby=0):
        self.x = startX
        self.y = startY
        self.goods = goods
        self.status = status
        self.mbx = mbx  #港口目标
        self.mby = mby
        self.mbgx = mbx  #
        self.mbgy = mby
        self.mbg_zhen = 0
        self.route=[]
        self.goods_route=[]
        self.route_goods_index = 0
        self.route_berth_index = 0
        self.flag = 0   #记录转换点
        self.move_history = []
        self.is_alive = 1   #这个机器人是活的

    def move2(self):
        if self.goods == 1:

            self.flag=0
            #不知道为什么需要这个 打个补丁
            if (self.x,self.y) in special_points:
                direction = wander(self.x,self.y)
                return direction
            if self.route == []:
                nearest_berth = find_nearestberth((self.x, self.y), berth)
                self.mbx = nearest_berth.x
                self.mby = nearest_berth.y
                berth_robotnum[(self.mbx,self.mby)] +=1

                # sys.stderr.write("当前的点在"+str(self.x)+" "+str(self.y)+"目标港口为"+str(nearest_berth.x)+str(nearest_berth.y)+"\n")
                self.route = path[(nearest_berth.x, nearest_berth.y)][(self.x, self.y)]
                self.route_berth_index = 0


            if self.route_berth_index > len(self.route) - 2:
                self.route = []
                self.route_berth_index = 0
                direction = A_star((self.x, self.y), (self.mbx, self.mby), self.move_history)
                self.move_history.append(((self.x, self.y), direction))
                return direction


            if distance((self.x,self.y),(self.mbx,self.mby)) < 20:
                direction = A_star((self.x,self.y),(self.mbx,self.mby),self.move_history)
                self.move_history.append(((self.x, self.y), direction))
                self.route=[]
                self.route_berth_index=0
                # if self.mbx <= self.x and self.x <=self.mbx+3 and self.mby<=self.y and self.y<=self.y+3:
                #     self.route = []
            else:
                direction = find_direction_according_to_route(self.route,self.route_berth_index)
                self.route_berth_index += 1

        if self.goods == 0:
            # if self.flag == 0:
            #     nearest_goods = find_nearestgoods((self.x,self.y),goods)
            #     #只分配给一个机器人
            #     if len(goods) >=2:
            #         goods.remove(nearest_goods)
            #     self.mbgx = nearest_goods[0][0]
            #     self.mbgy = nearest_goods[0][1]
            #     self.mbg_zhen=nearest_goods[1]
            #     self.flag = 1


            # if self.goods_route != []:
            #     direction = find_direction_according_to_route(self.goods_route,self.route_goods_index)
            #     self.route_goods_index+=1
            #     return direction
            # nearest_goods = find_nearestgoods((self.x,self.y),goods)
            # if nearest_goods != None:
            #     direction = A_star((self.x,self.y),(nearest_goods[0][0],nearest_goods[0][1]),self.move_history)
            #     self.move_history.append(((self.x, self.y), direction))
            # else:
            #     nearest_goods = goods[0]
            #     self.goods_route=dijkstra(mapp,(self.x,self.y),goods[0])
            #     self.route_goods_index=0
            #     goods.remove(nearest_goods)


            nearest_goods = find_nearestgoods((self.x,self.y),goods)
            if nearest_goods != None:
                direction = get_best_direction((self.x, self.y), (nearest_goods[0][0],nearest_goods[0][1]),self.move_history)
                self.move_history.append(((self.x,self.y),direction))
            else:
                direction = wander(self.x,self.y)

        if len(self.move_history) >=40 :
            self.move_history.pop(0)

        return direction

    #move 机器人的移动策略
    #ljr 3.7  晚上 尝试思路1：A*
    def move(self,i):
        #某个撞车的函数
        #撞车了要回溯  主要是感觉不太会装车吧
        # if self.status == 0:
        #     direction = opposite_direction(self.move_history[-1][1])
        #     self.move_history.append(((self.x,self.y),direction))
        #     if len(self.move_history) >= 125:
        #         self.move_history.pop(0)
        #     return direction

        #先判断是否装了货物
        #已经装了货物
        # if self.goods == 1:
        #     nearest_berth = find_nearestberth((self.x, self.y), berth)
        #     # self.mbx = nearest_berth.x
        #     # self.mby = nearest_berth.y
        #     # if self.flag == 0:
        #     #     self.flag = 1
        #
        #     if distance((self.x,self.y),(nearest_berth.x,nearest_berth.y))<40:
        #         direction=get_best_direction((self.x,self.y),(nearest_berth.x,nearest_berth.y),self.move_history)
        #         self.move_history.append(((self.x,self.y),direction))
        #         return direction
        #     else:
        #         length = 0
        #         if self.route_berth_index == 0:
        #             path,length=dijkstra(mapp,(self.x,self.y),(nearest_berth.x,nearest_berth.y))
        #             self.route=path
        #         if self.route:  # 确保route不为空
        #             direction = find_direction_according_to_route(self.route,self.route_berth_index)
        #             self.route_berth_index += 1
        #             #如果到了就清零
        #             if self.route_berth_index == length:
        #                 self.route_berth_index = 0
        #             return direction
        #         else:
        #             direction = get_best_direction((self.x, self.y), (nearest_berth.x, nearest_berth.y),self.move_history)
        #             # self.move_history.append(((self.x, self.y), direction))
        #
        #             return direction
        if self.goods == 1:
            if self.route == []:
                # nearest_berth = find_nearestberth((self.x, self.y), berth)
                nearest_berth = berth[i]
                self.mbx=nearest_berth.x
                self.mby=nearest_berth.y
                # if distance((self.x,self.y),(self.mbx,self.mby))>=20:
                self.route=dijkstra(mapp,(self.x,self.y),(self.mbx,self.mby))
                self.route_berth_index=0

            sys.stderr.write("hhhhhhhh"+str(self.route))
            if self.route_berth_index > len(self.route) - 2:
                self.route = []
                self.route_berth_index = 0
                direction = A_star((self.x, self.y), (self.mbx, self.mby), self.move_history)
                self.move_history.append(((self.x, self.y), direction))
                return direction

            if distance((self.x,self.y),(self.mbx,self.mby))<20:
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
        #未安装货物
        if self.goods == 0:
            # if self.goods_route != []:
            #     direction = find_direction_according_to_route(self.goods_route,self.route_goods_index)
            #     self.route_goods_index+=1
            #     return direction
            nearest_goods = find_nearestgoods((self.x,self.y),goods)
            if nearest_goods != None:
                direction = get_best_direction((self.x, self.y), (nearest_goods[0][0],nearest_goods[0][1]),self.move_history)
                self.move_history.append(((self.x,self.y),direction))
            else:
                direction=wander(self.x,self.y)
            # else:
            #     nearest_goods = goods[0]
            #     self.goods_route = dijkstra(mapp, (self.x, self.y), goods[0])
            #     self.route_goods_index = 0
            #
            #     sys.stderr.write(str(self.goods_route)+"\n")
            #
            #     direction =find_direction_according_to_route(self.goods_route,self.route_goods_index)
            #     self.route_goods_index+=1
            #     goods.remove(nearest_goods)



            # if distance((self.x,self.y),(nearest_goods[0][0],nearest_goods[0][1])) < 50:
            #     direction = get_best_direction((self.x, self.y), (nearest_goods[0][0],nearest_goods[0][1]),self.move_history)
            #     self.move_history.append(((self.x,self.y),direction))
            # else:
            #     if self.route_index == 0:
            #         path,length=dijkstra(mapp,(self.x,self.y),(nearest_goods[0][0],nearest_goods[0][1]))
            #         self.route = jiexi_path(path)
            #
            #     if self.route:  # 确保route不为空
            #         if 0 <= self.route_index < len(self.route):  # 检查索引是否在有效范围内
            #             direction = self.route[self.route_index]
            #             self.route_index +=1
            #     else:
            #         direction = get_best_direction((self.x, self.y), (nearest_goods[0][0],nearest_goods[0][1]),self.move_history)
            #         self.move_history.append(((self.x, self.y), direction))

      #具有一定的记忆功能 但得后续再改
        if len(self.move_history) >=40 :
            self.move_history.pop(0)

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

def Init(mapp):

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
        special_points.append((berth[id].x,berth[id].y))

        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]

        sys.stderr.write("港口的位置:"+str(berth[id].x)+" "+str(berth[id].y)+"\n")
    boat_capacity = int(input())

    #对mapp进行预处理
    mapp = retain_connected_special_points(mapp,special_points)
    sys.stderr.write(str(mapp[109][61]))
    for rt in robot:
        if mapp[rt.x][rt.y] == 0:
            rt.is_alive = 0

    okk = input()
    print("OK")
    sys.stdout.flush()

#ljr增加了一个删除货物的函数
def delete_goods(zhen):
    for i in range(len(goods)):
        if zhen - goods[i][1]>=1000:
            gds[goods[i][0][0]][goods[i][0][1]]=0
            goods.remove(goods[i])
        else:
           break

def Input(zhen):
    # 输入函数，用于在每一"帧"读取新的输入数据并更新系统状态
    id, money = map(int, input().split(" "))
    num = int(input())
    for i in range(num):
        x, y, val = map(int, input().split())
        #增加一步检测 若货物在死位则不增加
        if mapp[x][y] == 0:
            continue
        else:
            goods.append(((x,y),zhen))
            gds[x][y] = val

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
            return True
    return False


if __name__ == "__main__":
    # 主程序入口
    Init(mapp)  # 调用初始化函数
    # 创造一个线程单独来干迪杰斯特拉的活
    t1 = threading.Thread(target=create_route_memory)
    t1.start()
    sign=1
    sign2=1
    for zhen in range(1, 15001):
        sys.stderr.write(str(t1.is_alive()))
        # 主循环，模拟15000帧的运
        id = Input(zhen)  # 处理每帧的输入
        if zhen == 100:
            print("ship",0,3)
            print("ship",1,5)
            print("ship", 2, 4)
            print("ship", 3, 7)
            print("ship", 4, 8)
        if zhen == 12000:
            print("go",1)
            print("go", 4)
            print("go",2)
            print("go",3)
            print("go", 0)
        delete_goods(zhen)

        if t1.is_alive():  #当迪杰斯特拉整体没有算完的时候
            if sign2 == 1:
                for rt in robot:
                    rt.route=[]
                    rt.route_berth_index=0
            for i in range(robot_num):
                if mapp[robot[i].x][robot[i].y] == 0:
                    continue
                if is_robot_at_any_goods(robot[i], goods):
                    sys.stderr.write("找到一个")
                    print("get", i)
                    robot[i].route = []
                    robot[i].route_berth_index = 0
                    goods_go((robot[i].x,robot[i].y))

                if is_robot_at_any_port(robot[i], berth):
                    print("pull", i)

                    # if robot[i].flag == 1:
                    #     berth_robotnum[(robot[i].mbx, robot[i].mby)] -= 1
                    # robot[i].flag = 0

                    robot[i].route_goods_index= []
                    robot[i].route_goods_index = 0
                # 控制每个机器人移动
                print("move", i, robot[i].move(i))

                sign2=0
                sys.stdout.flush()

        else: #迪杰斯特拉算完之后 到港口直接使用迪杰斯特拉
            #第一次先初始化 去掉之前的痕迹
            if sign == 1:
                for i in range(robot_num):
                    robot[i].route=[]
                    robot[i].route_berth_index=0
            for i in range(robot_num):
                if mapp[robot[i].x][robot[i].y] == 0:
                    continue

                if is_robot_at_any_goods(robot[i], goods):
                    sys.stderr.write("找到一个")
                    print("get", i)
                    robot[i].route = []
                    robot[i].route_berth_index = 0
                    goods_go((robot[i].x,robot[i].y))

                if is_robot_at_any_port(robot[i], berth):
                    print("pull", i)

                    # 这里考虑到了密度 后面要改
                    # if robot[i].flag == 1:
                    #     berth_robotnum[(robot[i].mbx, robot[i].mby)] -= 1
                    # robot[i].flag = 0

                    robot[i].goods_route = []
                    robot[i].route_goods_index = 0

                # 控制每个机器人移动
                print("move", i, robot[i].move2())



                sys.stdout.flush()
                sign = 0
        print("OK")
        sys.stdout.flush()
