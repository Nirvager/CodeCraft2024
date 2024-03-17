import sys
import random
import numpy as np
from queue import PriorityQueue
from collections import deque
import threading

# 输出至文件
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
        self.goods_num = 0 # 储存泊位处堆积货物数
        self.berth_id = 0 # 泊位ID
        self.status = 0 # 泊位处是否有船 0无1有

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
    path2=path[::-1]
    # path.reverse()   这里应该不用倒转了
    path_length = len(path) - 1
    return path,path_length,path2
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
    # goals = [tuple(point) for point in np.argwhere(mapp == 1) if tuple(point) not in special_points]
    # for i in range(0, 10, 2):
    #     path[(berth[i].x,berth[i].y)]= dijkstra_to_multiple_goals_with_path_and_length(mapp, special_points[i], goals)
    #     path[(berth[i+1].x,berth[i+1].y)] = dijkstra_to_multiple_goals_with_path_and_length(mapp, special_points[i + 1], goals)
    goals = [tuple(point) for point in np.argwhere(mapp == 1) if tuple(point) not in special_points]
    for i in range(0, 4, 2):
        path[(special_points[i][0],special_points[i][1])]= dijkstra_to_multiple_goals_with_path_and_length(mapp, special_points[i], goals)
        path[(special_points[i+1][0],special_points[i+1][1])] = dijkstra_to_multiple_goals_with_path_and_length(mapp, special_points[i + 1], goals)
    path[(special_points[4][0],special_points[4][1])] = dijkstra_to_multiple_goals_with_path_and_length(mapp, special_points[4], goals)

    # # # sys.stderr.write(str(i)+"现在是"+str(i+1)+"\n")

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
def find_direction_according_to_route(route,index):
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




#计算相反方向 备用
def opposite_direction(direction):
    opposites = {2:3,3:2,1:0,0:1}
    return opposites.get(direction)
#ljr定义一下曼哈顿距离 后续可修改
def distance2(a, b) :
    return abs(a[0]-b[0])+abs(a[1]-b[1])
#采用这个距离
def distance(a,b):
    return np.sqrt(np.square(a[0]-b[0])+np.square(a[1]-b[1]))


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

#可以增加一个启发式函数
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
        if cost < min_cost and not is_circulate_movement(move_history,(current_pos,direction)):
        # if cost < min_cost:
            min_cost=cost
            best_direction=direction

    #如果被限制了 则wander一下
    if best_direction not in range(0,4):
        best_direction = wander(current_pos[0],current_pos[1])
    return best_direction
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
        if distance(good[0],robot_pos) <= 5:
            nearest_goods = good
            break
    # nearest_goods = min(goods,key=lambda good_pos:distance(robot_pos,good_pos[0]))
    return nearest_goods

def find_bestgoods(robot_pos, goods, current_frame):
    # 过滤掉在机器人到达前会消失的商品
    available_goods = [good for good in goods if good[1] + 1000 > current_frame + distance(robot_pos,good[0])]

    if len(available_goods) > 0:
        # 在剩余的商品中找到单位距离价值最大的商品
        max_value_goods = max(available_goods, key=lambda good: good[2] / distance(robot_pos,good[0]))
        return max_value_goods
    else:
        return None


#这个可以淘汰掉
# def find_secondnearestgoods(robot_pos, goods):
#     if len(goods) > 0:
#         # 计算每个商品的单位距离价值（val/distance），并找出这个值最大的商品
#         max_value_goods = max(goods, key=lambda good: good[2] / distance(robot_pos, good[0]))
#         return max_value_goods
#     else:
#         return None
#找最近的港口  这个可以改 根据调度策略
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


def check_is_good_alive(good_pos):
    for good in goods:
        if good[0]==good_pos:
            return True
    return False

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
        self.berth_id = -1 # 记录放下货物处泊位的ID


    def search_goods_after_dijie(self,dijiecnt):
        tmp_good = find_nearestgoods((self.x, self.y), goods)  # 这是为了应对 假如迪杰斯特拉很远 在路程中碰见很近的可以转换目标
        if tmp_good != None:
            if distance(tmp_good[0], (self.x, self.y)) < distance((self.x, self.y), (self.mbgx, self.mbgy)):
                # # # sys.stderr.write("我杀死了dd")
                self.route_goods_index = 0
                self.goods_route = []
                self.mbgx = 0
                self.mbgy = 0
                direction = get_best_direction((self.x, self.y), (tmp_good[0][0], tmp_good[0][1]), self.move_history)
                self.move_history.append(((self.x, self.y), direction))
                if len(self.move_history) >= 10:
                    self.move_history.pop(0)
                return direction

        # # 判断一下目标货物是不是消失了
        if check_is_good_alive((self.mbgx, self.mbgy)) == False:
            self.route_goods_index = 0
            self.goods_route = []
            self.mbgx = 0
            self.mbgy = 0

        if self.goods_route == []:
            nearest_berth = berth[(i // 2) * 2]
            # 没到港口点 但是非常近
            if (self.x, self.y) != (nearest_berth.x, nearest_berth.y) and distance((self.x, self.y), (nearest_berth.x, nearest_berth.y)) <= 8:
                direction = A_star((self.x, self.y), (nearest_berth.x, nearest_berth.y), self.move_history)
                self.move_history.append(((self.x, self.y), direction))
                if len(self.move_history) >= 5:
                    self.move_history.pop(0)
                return direction
            # 到了港口点 直接找路径
            elif (self.x, self.y) == (nearest_berth.x, nearest_berth.y):
                nearest_goods = find_bestgoods((self.x, self.y), goods, zhen)
                if nearest_goods == None:
                    direction = wander(self.x, self.y)
                    return direction
                self.goods_route = path[(self.x, self.y)][nearest_goods[0]][2]
                self.mbgx = nearest_goods[0][0]
                self.mbgy = nearest_goods[0][1]
                self.mbg_zhen = nearest_goods[1]
                self.mbg_val = nearest_goods[2]
                self.route_goods_index = 0
                # goods.remove(nearest_goods)   #这个地方要注意
            # 离港口点很远 直接迪杰斯特拉 主要是为了应对 交接时段以及物品被抢占
            else:
                nearest_goods = find_bestgoods((self.x, self.y), goods, zhen)
                if nearest_goods == None:
                    direction = wander(self.x, self.y)
                    return direction
                # 太远了我们还是A*一下 减少迪杰斯特拉的时间
                if distance(nearest_goods[0], (self.x, self.y)) >= 25 or dijiecnt >=1:
                    self.mbgx=0
                    self.mbgy=0
                    # 这个地方可以修改
                    # better_nearest_goods = half_nearest(nearest_goods)
                    direction = get_best_direction((self.x, self.y), nearest_goods[0], self.move_history)
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
                dijiecnt+=1
                self.route_goods_index = 0
            # 以防万一index哪里出错了
        if self.route_goods_index > len(self.goods_route) - 2:
            self.goods_route = []
            self.route_goods_index = 0
            direction = wander(self.x,self.y)
            return direction

        direction = find_direction_according_to_route(self.goods_route, self.route_goods_index)
        self.route_goods_index += 1
        return direction

    def search_berth_after_dijie(self,i):
        # 不知道为什么需要这个 打个补丁
        if (self.x, self.y) in special_points:
            direction = wander(self.x, self.y)
            return direction

        if self.route == []:
            # nearest_berth = find_nearestberth((self.x, self.y), berth)
            nearest_berth = berth[(i // 2) * 2]
            self.mbx = nearest_berth.x
            self.mby = nearest_berth.y
            berth_robotnum[(self.mbx, self.mby)] += 1

            # # # sys.stderr.write("当前的点在"+str(self.x)+" "+str(self.y)+"目标港口为"+str(nearest_berth.x)+str(nearest_berth.y)+"\n")
            self.route = path[(nearest_berth.x, nearest_berth.y)][(self.x, self.y)][0]
            self.route_berth_index = 0

        if self.route_berth_index > len(self.route) - 2:
            self.route = []
            self.route_berth_index = 0
            direction = A_star((self.x, self.y), (self.mbx, self.mby), self.move_history)
            self.move_history.append(((self.x, self.y), direction))
            if len(self.move_history) >= 10:
                self.move_history.pop(0)
            return direction

        direction = find_direction_according_to_route(self.route, self.route_berth_index)
        self.route_berth_index += 1
        return direction
    def search_goods(self,dijiecnt):
        tmp_good = find_nearestgoods((self.x,self.y), goods)    #这是为了应对 假如迪杰斯特拉很远 在路程中碰见很近的可以转换目标
        if tmp_good != None:
            if distance(tmp_good[0],(self.x,self.y)) < distance((self.x,self.y),(self.mbgx,self.mbgy)):
                # # # sys.stderr.write("我杀死了dd")
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
            nearest_goods = find_bestgoods((self.x, self.y), goods, zhen)
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

            # # # sys.stderr.write("我找到了缔结斯特勒")
            self.route_goods_index = 0


            # 以防万一index哪里出错了
        if self.route_goods_index > len(self.goods_route) - 2:
            self.goods_route = []
            self.route_goods_index = 0
            direction = A_star((self.x, self.y), (self.mbgx, self.mbgy), self.move_history)
            self.move_history.append(((self.x, self.y), direction))
            if len(self.move_history) >= 20:
                self.move_history.pop(0)
            return direction

        direction = find_direction_according_to_route(self.goods_route, self.route_goods_index)
        self.route_goods_index += 1
        return direction

    def search_berth(self,i,dijiecent):
        if self.route == []:
            # nearest_berth = find_nearestberth((self.x, self.y), berth)
            nearest_berth = berth[(i // 2) * 2]
            self.mbx = nearest_berth.x
            self.mby = nearest_berth.y
            # if distance((self.x,self.y),(self.mbx,self.mby))>=20:
            self.route = dijkstra(mapp, (self.x, self.y), (self.mbx, self.mby))
            self.route_berth_index = 0

        # # # sys.stderr.write("hhhhhhhh"+str(self.route))
        if self.route_berth_index > len(self.route) - 2:
            self.route = []
            self.route_berth_index = 0
            direction = A_star((self.x, self.y), (self.mbx, self.mby), self.move_history)
            self.move_history.append(((self.x, self.y), direction))
            if len(self.move_history) >= 10:
                self.move_history.pop(0)
            return direction

        if self.route:  # 确保route不为空
            direction = find_direction_according_to_route(self.route, self.route_berth_index)
            self.route_berth_index += 1
        else:
            direction = A_star((self.x, self.y), (self.mbx, self.mby), self.move_history)
            self.move_history.append(((self.x, self.y), direction))
        if len(self.move_history) >= 10:
            self.move_history.pop(0)
        return direction
    #move2是迪杰斯特拉 港口对全图嵌入后的移动策略
    def move2(self,i,dijiecnt):
        if self.goods == 1:
            direction = self.search_berth_after_dijie(i)

        if self.goods == 0:
            direction = self.search_goods_after_dijie(dijiecnt)
        return direction
    #move是dijie未结束时的策略
    def move(self,i,dijiecnt):
        if self.goods == 1:
            direction = self.search_berth(i,dijiecnt)

        #未安装货物
        if self.goods == 0:
            direction = self.search_goods(dijiecnt)
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
        self.start_zhen = 0
        self.arrive_zhen = 0


# 创建船实例列表
boat = [Boat() for _ in range(10)]

# 更多全局变量定义
money = 0  # 货币数量
boat_capacity = 0  # 船的容量
id = 0
ch = []  # 可能用于存储地图信息   可以修改感觉
gds = [[0 for _ in range(N)] for _ in range(N)]  # 二维数组，用于存储货物信息
sorted_berth_goods_num = [] # 按堆积货物数对泊位排序

def Init(mapp,t1):

    # 初始化函数，用于从输入读取初始配置
    global boat_capacity
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
        if i % 2 == 0:
            special_points.append((berth[id].x, berth[id].y))

        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
        berth[id].berth_id = id

        # # sys.stderr.write("港口的位置:"+str(berth[id].x)+" "+str(berth[id].y)+"\n")
    boat_capacity = int(input())
    sys.stderr.write("芝士船舶容量: " + str(boat_capacity) + "\n")

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
#去除一些位置不好的货物   这个有待商榷
def check_goods(x,y):
    if mapp[x][y] == 0:
        return False
    cnt = 0
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        if mapp[x+dx][y+dy] == 0:
            cnt +=1
    if cnt>=3:
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

temp_money = 0 # 用于调试 见下
money_flag = 0

def Input(zhen):
    # 输入函数，用于在每一"帧"读取新的输入数据并更新系统状态
    id, money = map(int, input().split(" "))

    # 调试用 输出每一帧的钱
    global temp_money
    if money == 0:
        pass
    else:
        if temp_money < 4:
            sys.stderr.write("\t芝士第" + str(id) + "帧的money: " + str(money))
            temp_money += 1
        else:
            sys.stderr.write("芝士第" + str(id) + "帧的money: " + str(money) + "\n")
            temp_money = 0

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
# 后续可能会用到机器人pull货物的泊位的ID
def int_is_robot_at_any_port(robot, berth):
    t = 1
    for port in berth:
        if port.x <= robot.x and robot.x <=port.x+3 and port.y<=robot.y and robot.y <=port.y+3:
            return t
        t += 1
    return 0

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


#通过当前位置和方向计算下一个位置
def use_direction_to_calculate_nextpos(x,y,direction):
    if direction == 3:
        return (x+1,y)
    elif direction == 2:
        return (x-1,y)
    elif direction == 1:
        return (x,y-1)
    elif direction == 0:
        return (x,y+1)
    else:
        return (x,y)


#记录机器人本轮移动前位置
list_current_pos=[]
#记录机器人移动后位置
list_next_pos=[]
#记录机器人本轮移动方向
direction_list=[]
#判断目的地有没有相同的
def check_if_there_is_same_next_pos_before(i):
    # # # sys.stderr.write(str(list_next_pos)+str(i))
    tmp = list_next_pos[i]
    for j in range(i):
        if list_next_pos[j] == tmp:
            return True
    return False
def check_if_there_is_a_duizhuangcrash_before(i):
    tmp = list_next_pos[i]
    for j in range(i):
        if list_current_pos[j] == tmp:
            tmp2 = list_next_pos[j]
            if tmp2 == list_current_pos[i]:
                return True
    return False
def check_if_there_is_a_robot(x,y,list_next_pos,i):
    for j in range(i):
        if list_next_pos[j] == (x,y):
            return True
    return False
def check_another_possible_direction(x,y,direction,list_next_pos,i):
    if direction == 3 or direction ==2 :
        if mapp[x][y+1] == 1  and not check_if_there_is_a_robot(x,y+1,list_next_pos,i):
            return 0,(x,y+1)
        if mapp[x][y-1] == 1 and not check_if_there_is_a_robot(x,y-1,list_next_pos,i):
            return 1,(x,y-1)
        if direction == 3 and not check_if_there_is_a_robot(x-1,y,list_next_pos,i):
            return 2,(x-1,y)
        if direction == 2 and not check_if_there_is_a_robot(x+1,y,list_next_pos,i):
            return 3,(x+1,y)

    if direction == 0 or direction == 1:
        if mapp[x+1][y] == 1  and not check_if_there_is_a_robot(x+1,y,list_next_pos,i):
            return 3,(x+1,y)
        if mapp[x-1][y] == 1  and not check_if_there_is_a_robot(x-1,y,list_next_pos,i):
            return 2,(x-1,y)
        if direction == 0 and not check_if_there_is_a_robot(x,y-1,list_next_pos,i):
            return 1,(x,y-1)
        if direction == 1 and not check_if_there_is_a_robot(x,y+1,list_next_pos,i):
            return 0,(x,y+1)
if __name__ == "__main__":
    # 主程序入口
    t1 = threading.Thread(target=create_route_memory)
    Init(mapp,t1)  # 调用初始化函数
    # 创造一个线程单独来干迪杰斯特拉的活
    sign=1   #用来标记第一次进入t1.is_dead 便于初始化
    # sign2=1   #用来标记第一次进入t1.is_alive 便于初始化
    for zhen in range(1, 15001):
        delete_goods(zhen)
        # # sys.stderr.write(str(t1.is_alive()))
        # 主循环，模拟15000帧的运
        id = Input(zhen)  # 处理每帧的输入

        # 每一次进来先初始化
        list_current_pos = []
        list_next_pos = []
        direction_list = []

        if t1.is_alive():  #当迪杰斯特拉整体没有算完的时候
            dijiecnt = 0
            for i in range(robot_num):

                list_current_pos.append((robot[i].x, robot[i].y))
                if robot[i].status == 0:
                    list_next_pos.append((robot[i].x, robot[i].y))
                    direction_list.append(-2)
                    continue
                if mapp[robot[i].x][robot[i].y] == 0:
                    list_next_pos.append((robot[i].x, robot[i].y))
                    #表示不动
                    direction_list.append(-2)
                    continue
                if is_robot_at_any_goods(robot[i], goods):
                    # # sys.stderr.write("找到一个")
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

                if int_is_robot_at_any_port(robot[i], berth) > 0:
                    berth[int_is_robot_at_any_port(robot[i], berth) - 1].goods_num += 1
                    print("pull", i)

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
                direction = robot[i].move(i,dijiecnt)
                direction_list.append(direction)

                next_pos = use_direction_to_calculate_nextpos(robot[i].x,robot[i].y,direction)
                list_next_pos.append(next_pos)
                sys.stdout.flush()
            for i in range(robot_num):
                if check_if_there_is_same_next_pos_before(i) == True:
                    #停下来
                    if robot[i].route:
                        robot[i].route_berth_index -= 1
                    if robot[i].goods_route:
                        robot[i].route_goods_index -=1
                    list_next_pos[i] = list_current_pos[i]
                    continue
                if check_if_there_is_a_duizhuangcrash_before(i) == True:
                    direction_list[i],list_next_pos[i] =check_another_possible_direction(robot[i].x,robot[i].y,direction_list[i],list_next_pos,i)
                    if robot[i].route:
                        robot[i].route=[]
                        robot[i].route_berth_index=0
                        robot[i].move_history=[]
                    if robot[i].goods_route:
                        robot[i].goods_route=[]
                        robot[i].route_goods_index=0
                        robot[i].move_history = []
                if direction_list[i] in range(0, 4):
                    print("move", i, direction_list[i])
        else: #迪杰斯特拉算完之后 到港口直接使用迪杰斯特拉
            #第一次先初始化 去掉之前的痕迹
            if sign == 1:
                for i in range(robot_num):
                    robot[i].route=[]
                    robot[i].route_berth_index=0
            dijiecnt = 0
            for i in range(robot_num):
                list_current_pos.append((robot[i].x, robot[i].y))
                if robot[i].status == 0:
                    list_next_pos.append((robot[i].x, robot[i].y))
                    direction_list.append(-2)
                    continue
                if mapp[robot[i].x][robot[i].y] == 0:
                    list_next_pos.append((robot[i].x, robot[i].y))
                    #表示不动
                    direction_list.append(-2)
                    continue

                if is_robot_at_any_goods(robot[i], goods):
                    if robot[i].goods == 0:
                        # sys.stderr.write("找到一个")
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

                if int_is_robot_at_any_port(robot[i], berth) > 0:
                    if robot[i].goods == 1:
                        berth[int_is_robot_at_any_port(robot[i], berth) - 1].goods_num += 1
                        print("pull", i)
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
                direction = robot[i].move2(i,dijiecnt)
                direction_list.append(direction)

                next_pos = use_direction_to_calculate_nextpos(robot[i].x, robot[i].y, direction)
                list_next_pos.append(next_pos)
                sys.stdout.flush()

            for i in range(robot_num):
                if check_if_there_is_same_next_pos_before(i) == True:
                    # 停下来
                    if robot[i].route:
                        robot[i].route_berth_index -= 1
                    if robot[i].goods_route:
                        robot[i].route_goods_index -= 1
                    list_next_pos[i] = list_current_pos[i]
                    continue
                if check_if_there_is_a_duizhuangcrash_before(i) == True:
                    direction_list[i],list_next_pos[i] =check_another_possible_direction(robot[i].x,robot[i].y,direction_list[i],list_next_pos,i)
                    if robot[i].route:
                        robot[i].route=[]
                        robot[i].route_berth_index=0
                        robot[i].move_history=[]
                    if robot[i].goods_route:
                        robot[i].goods_route=[]
                        robot[i].route_goods_index=0
                        robot[i].move_history = []
                if direction_list[i] in range(0,4):
                    print("move", i, direction_list[i])
                sys.stdout.flush()

        # 船舶指令————暂不考虑同一帧执行多条指令
        stderr_flag_1000 = 0 # 调试用 用于输出1000帧时的泊位货物数
        for i in range(5):
            if zhen < 1000:
                # 初始给予1000ms运货
                continue
            else: # 正式开始

                # 调试用 循环到1000帧时输出十个泊位的货物数---------------------------------------------------------------------------------------------------------
                if zhen == 1000 and stderr_flag_1000 == 0:
                    sys.stderr.write("芝士1000帧时泊位装的货:" + "\n0: " + str(berth[0].goods_num) + "\t1: " + str(berth[1].goods_num) + "\t2: " + str(berth[2].goods_num) + "\t3: " + str(berth[3].goods_num) + "\t4: " + str(berth[4].goods_num) + "\n5: " + str(berth[5].goods_num) + "\t6: " + str(berth[6].goods_num) + "\t7: " + str(berth[7].goods_num) + "\t8: " + str(berth[8].goods_num) + "\t9: " + str(berth[9].goods_num))
                    stderr_flag_1000 = 1
                    pass
                # -----------------------------------------------------------------------------------------------------------------------------------------------

                if boat[i].status == 0:
                    # 运输中
                    if boat[i].pos == -1:
                        # 前往虚拟点 不打断此过程
                        # 此处需要后面的程序保证此船为装满、快装满、最终期限其中之一
                        continue
                    else: # boat[i].pos = range(10)
                        # 后续程序中提到的优化点 泊位间流动
                        pass # 先占位
                elif boat[i].status == 1:
                    # 正常工作
                    if boat[i].pos == -1:
                        # 位于虚拟点 等待发船
                        # 前往当前场上货物堆积最多的空闲泊位 这意味着需要在Berth类中增设状态属性

                        # 获取货物数最多的空闲泊位
                        no_boat_berths = [b for b in berth if not b.status]
                        sorted_berth_goods_num = sorted(no_boat_berths, key=lambda berth: berth.goods_num, reverse=True)

                        # 调试用 输出每次的no_boat_berths的berth_id-------------------------------------------------------------------------------
                        for b in sorted_berth_goods_num:
                            sys.stderr.write("\n芝士ship前排序的空闲港口货物数: 泊位ID(" + str(b.berth_id) + ") 剩余货物数: " + str(b.goods_num))
                        # -----------------------------------------------------------------------------------------------------------------------

                        max_goods_berth = sorted_berth_goods_num[0]

                        # 更新泊位及船舶状态信息
                        # PS: Boat类中的status和pos由判题器输入 无需手动更新
                        max_goods_berth.status = 1 # 泊位非空闲
                        print("ship", i, max_goods_berth.berth_id)

                        # 调试用 输出程序发出的ship指令类型及参数----------------------------------------------------------------------------------
                        sys.stderr.write("\n" + str(zhen) + ": 从虚拟点出发的ship " + str(i) + " " + str(max_goods_berth.berth_id) + "\t\t")
                        sys.stderr.write("ship后berth.status: " + str(max_goods_berth.status))
                        # -----------------------------------------------------------------------------------------------------------------------

                    else: # boat[i].pos == range(10)
                        # 位于泊位处 装货中
                        """此时有以下情况
                        1. 船装满了 直接返回虚拟点
                        2. 船没装满
                            a. 泊位空了 去往其他泊位 选取货物最多的空闲泊位
    ###########################优化点: 可以加入时间判断 在自身到达非空泊位时原本的船能否出来 同时在泊位之间流动的过程中可能会有其他泊位货物数更多...... 等待发掘##
                            b. 泊位未空 继续装船
                            c. 大限将至 速速返回
                        """
                        if 15000 - berth[boat[i].pos].transport_time == zhen:
                            # 保证程序结束时所有船上装载的货物都有效 成功运回虚拟点
                            # 此时已经结束 同时有上面status==0和pos==-1部分的保证 船舶不会再乱跑 故无需更改船舶的运行状态
                            berth[boat[i].pos].status = 0 # 泊位空闲
                            boat[i].num = 0 # 手动更新船舶装载货物数
                            print("go", i)

                            # 调试用 输出程序发出的go指令的类型及参数--------------------------------------------------
                            sys.stderr.write("\n" + str(zhen) + ": 大限将至的go " + str(i))
                            # ---------------------------------------------------------------------------------------

                            continue
                        if boat[i].num == boat_capacity:
                            # 船已装满
                            # 更新泊位及船舶状态信息
                            # PS: Boat类中的status和pos由判题器输入 无需手动更新
                            berth[boat[i].pos].status = 0 # 泊位空闲
                            boat[i].num = 0 # 手动更新船舶装载货物数
                            print("go", i)

                            # 调试用 输出程序发出的go指令的类型及参数--------------------------------------------------
                            sys.stderr.write("\n" + str(zhen) + ": 船已装满的go " + str(i))
                            # ---------------------------------------------------------------------------------------

                        else: # boat[i].num < boat_capacity
                            if berth[boat[i].pos].goods_num == 0:
                                # 前往其他泊位 需要考虑当前状态下泊位的具体情况
    ##############################优化点: 在船快装满时无需花费500帧前往其他泊位 直接go 性价比更高###############################################################
                                
                                # 获取货物数最多的空闲泊位
                                no_boat_berths = [b for b in berth if not b.status]
                                sorted_berth_goods_num = sorted(no_boat_berths, key=lambda berth: berth.goods_num, reverse=True)


                                # 调试用 输出每次的no_boat_berths的berth_id-------------------------------------------------------------------------------
                                for b in sorted_berth_goods_num:
                                    sys.stderr.write("\n芝士ship前排序的空闲港口货物数: 泊位ID(" + str(b.berth_id) + ") 剩余货物数: " + str(b.goods_num))
                                # -----------------------------------------------------------------------------------------------------------------------

                                max_goods_berth = sorted_berth_goods_num[0]
                                # 更新泊位及船舶状态信息
                                # PS: Boat类中的status和pos由判题器输入 无需手动更新
                                max_goods_berth.status = 1 # 泊位非空闲
                                print("ship", i, max_goods_berth.berth_id)

                                # 调试用 输出程序发出的go指令的类型及参数----------------------------------------------------------------------
                                sys.stderr.write("\n" + str(zhen) + ": 从泊位出发的ship " + str(i) + " " + str(max_goods_berth.berth_id))
                                # -----------------------------------------------------------------------------------------------------------

                            else: # berth[boat[i].pos].goods_num > 0
                                # 继续装货
    ##############################优化点: 可采用cz同志的停留帧数x装载速度求出最终货物数 不需要每一次循环都执行本条语句############################################
                                boat[i].num += min(min(berth[boat[i].pos].loading_speed, berth[boat[i].pos].goods_num), boat_capacity - boat[i].num)
                else: # boat[i].status == 2
                    # 泊位外等待 出现在两艘船决定去往一个点时
                    """此处情况暂且不考虑 可能会发生在某一泊位货物堆积特别多 多到一艘船装不完时
                    实践证明 该情况很有可能发生 而且很重要
                    """
                    pass # 占位
            
            # if boat[i].status == 1 and boat[i].pos != -1:
            #     berth[boat[i].pos].goods_num -= min(berth[boat[i].pos].loading_speed, berth[boat[i].pos].goods_num)
            # # 首次发船
            # if zhen == 1000:
            #     sorted_berth_goods_num = sorted(berth, key=lambda berth: berth.goods_num, reverse=True) # 对100帧时的泊位进行货物数的降序排列
            #     boat[i].start_zhen = 1000
            #     print("ship", i, sorted_berth_goods_num[i].berth_id)
            # # 回到虚拟点直接发船
            # if boat[i].status == 1 and boat[i].pos == -1 and zhen > 1000:
            #     sorted_berth_goods_num = sorted(berth, key=lambda berth: berth.goods_num, reverse=True) # 对100帧时的泊位进行货物数的降序排列
            #     print("ship", i, sorted_berth_goods_num[i].berth_id)


            # if zhen == 3500:
            #     print("go", i)
            # if zhen == 8500:
            #     print("go", i)
            # # if zhen == 15000 - berth[boat[i].pos].transport_time:
            # if zhen == 11500:
            #     print("go", i)

        print("OK")
        sys.stdout.flush()
    # 调试用 输出程序循环结束后各个泊位的剩余货物数--------------------------------------------------------------------------------------------
    sys.stderr.write("\n泊位剩余货物数:\n0: " + str(berth[0].goods_num) + "\t1: " + str(berth[1].goods_num) + "\t2: " + str(berth[2].goods_num) + "\t3: " + str(berth[3].goods_num) + "\t4: " + str(berth[4].goods_num) + "\n5: " + str(berth[5].goods_num) + "\t6: " + str(berth[6].goods_num) + "\t7: " + str(berth[7].goods_num) + "\t8: " + str(berth[8].goods_num) + "\t9: " + str(berth[9].goods_num) + "\n")
    # --------------------------------------------------------------------------------------------------------------------------------------
