import sys
import random
import numpy as np
from queue import PriorityQueue

# 定义全局变量
n = 200  # 地图的尺寸或某个特定的数量标志
robot_num = 10  # 机器人的数量
berth_num = 10  # 泊位的数量
N = 210  # 用于创建二维数组时的尺寸，略大于n，可能是为了处理边界情况

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

#ljr 3.9初步尝试将A*和Dijskra结合
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
    if start == goal:
        return [start], 0  # 起点即目标点，路径长度为0
    if goal not in predecessors:
        return [], float('inf')  # 目标点不可达

    path = [goal]
    current = goal
    while current in predecessors:
        current = predecessors[current]
        path.append(current)
        if current == start:
            break
    path.reverse()
    path_length = len(path) - 1
    return path, path_length

def dijkstra(grid, start, goal):
    """
    使用Dijkstra算法找到从起点到多个目标点的最短路径及其长度。

    参数:
    grid: 二维数组表示的地图，1表示可通行，0表示墙壁。
    start: 起点坐标(tuple)。
    goals: 多个目标点的列表(list of tuples)。

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
    path,length = reconstruct_path(predecessors,start,goal)
    return path,length

#ljr3_8 尝试追踪一下每个点的运动

def is_repetitive_movement(move_history, choice):
    """检查重复模式"""
    if len(move_history) <=3:
        return False
    if choice[1] == opposite_direction(move_history[-1][1]):
        return True
    return False

#计算相反方向 备用
def opposite_direction(direction):
    opposites = {2:3,3:2,1:0,0:1}
    return opposites.get(direction)

#ljr定义一下曼哈顿距离 后续可修改
def distance(a, b) :
    return abs(a[0]-b[0])+abs(a[1]-b[1])

#找最近的货物
def find_nearestgoods(robot_pos, goods):
    nearest_goods = min(goods, key=lambda goods_pos:abs(goods_pos[0][0]-robot_pos[0])+abs(goods_pos[0][1]-robot_pos[1]))
    return nearest_goods

#找最近的港口
def find_nearestberth(robot_pos, berth):
    nearest_berth = min(berth,key=lambda tmp:abs(tmp.x-robot_pos[0])+abs(tmp.y-robot_pos[1]))
    return nearest_berth

#避免陷入死角
def is_good(next_pos,dx,dy):
    if ch[next_pos[0]+dx][next_pos[1]+dy] == '.' or ch[next_pos[0]-dx][next_pos[1]+dy] == '.' or ch[next_pos[0]+dx][next_pos[1]-dy] == '.':
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
        if (ch[next_pos[0]][next_pos[1]] == '.' or ch[next_pos[0]][next_pos[1]] == 'B') and not is_repetitive_movement(move_history,(current_pos,direction)) and is_good(next_pos,dx,dy):
            feasible_directions.append((direction,next_pos))

    return feasible_directions

def is_circulate_movement(move_history,choice):
    if choice in move_history:
        return True
    return False

def heuristic(next_pos,goal_pos):
    base_cost = abs(next_pos[0] - goal_pos[0]) + abs(next_pos[1] - goal_pos[1])
    # 如果已经很接近目标，可能不需要调整成本
    if base_cost < 120:
        return base_cost

    obstacle_cost = 0
    dx = [1, -1, 0, 0]
    dy = [0, 0, 1, -1]
    for i in range(4):
        new_x, new_y = next_pos[0] + dx[i], next_pos[1] + dy[i]
        # 确保不会检查地图边界之外的位置
        if 0 <= new_x < len(ch) and 0 <= new_y < len(ch[0]) and ch[new_x][new_y] != '.':
            obstacle_cost += 0.5  # 对于每个障碍物增加额外的成本

    # 动态调整障碍物成本，距离目标越远，障碍物影响越小
    adjusted_obstacle_cost = obstacle_cost * (5 / (base_cost + 1))

    return base_cost + adjusted_obstacle_cost

def get_best_direction(current_pos,goal_pos,move_history):
    """基于当前位置、目标位置和障碍物，决定最优的单步移动方向"""
    feasible_directions = get_feasible_directions(current_pos,move_history)

    #计算每个可行方向的启发式成本
    best_direction = 0
    min_cost = float('inf')
    for direction,next_pos in feasible_directions:
        #启发式成本必改
        cost = heuristic(next_pos,goal_pos)
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

goal_berth=[]

#ljr3.8判断一下机器人是否在港口
def is_robot_at_any_port(robot, berth):
    for port in berth:
        if robot.x == port.x and robot.y == port.y:
            return True
    return False

# 后续可能会用到机器人pull货物的泊位的ID
def int_is_robot_at_any_port(robot, berth):
    for i in range(0, 5):
        if berth[i].x <= robot.x <= (berth[i].x + 3) and berth[i].y <= robot.y <= (berth[i].y + 3):
            return i+1
    return 0

# 货物删除函数尚未连接至Robot类中的相应函数中
#ljr增加了一个删除货物的函数
def delete_goods(zhen):
    for i in range(len(goods)):
        if zhen - goods[i][1]>=1000:
            gds[goods[i][0][0]][goods[i][0][1]]=0
            goods.remove(goods[i])
        else:
           break

# 类定义
class Robot:
    # 初始化机器人的属性：位置、携带的货物、状态、目标泊位的坐标
    def __init__(self, startX=0, startY=0, goods=0, status=0, mbx=0, mby=0):
        self.x = startX
        self.y = startY
        self.goods = goods
        self.status = status
        self.mbx = mbx
        self.mby = mby
        self.move_history=[]
        self.route=[]
        self.route_index = 0

    #move 机器人的移动策略
    #ljr 3.7  晚上 尝试思路1：A*
    def move(self):
        #撞车了要回溯
        if self.status == 0:
            direction = opposite_direction(self.move_history[-1][1])
            self.move_history.append(((self.x,self.y),direction))
            if len(self.move_history) >= 125:
                self.move_history.pop(0)
            return direction

        #先判断是否装了货物
        #已经装了货物
        if self.goods == 1:

            # nearest_berth = find_nearestberth((self.x,self.y),berth)

            nearest_berth = find_nearestberth((self.x, self.y), berth)
            if distance((self.x,self.y),(nearest_berth.x,nearest_berth.y))<100:
                direction=get_best_direction((self.x,self.y),(nearest_berth.x,nearest_berth.y),self.move_history)
                self.move_history.append(((self.x,self.y),direction))
            else:
                if self.route_index == 0:
                    path,length=dijkstra(mapp,(self.x,self.y),(nearest_berth.x,nearest_berth.y))
                    self.route = jiexi_path(path)


                if self.route:  # 确保route不为空
                    if 0 <= self.route_index < len(self.route):  # 检查索引是否在有效范围内
                        direction = self.route[self.route_index]
                        self.route_index +=1
                else:
                    direction = get_best_direction((self.x, self.y), (nearest_berth.x, nearest_berth.y),
                                                   self.move_history)
                    self.move_history.append(((self.x, self.y), direction))

                return direction


        #未安装货物
        if self.goods == 0:
            nearest_goods = find_nearestgoods((self.x,self.y),goods)
            direction = get_best_direction((self.x, self.y), (nearest_goods[0][0],nearest_goods[0][1]),self.move_history)
            self.move_history.append(((self.x,self.y),direction))

        if len(self.move_history) >=180 :
            self.move_history.pop(0)

        return direction
    
    def get(self, id):
        self.goods = 1 # robot货物状态更新 持有货物
        # 在地图中删除该点中记录的货物
        # delete_goods
        print("get", id)

    def pull(self, id):
        self.status = 0
        # 泊位货物数+1
        berth[temp-1].g_num += 1
        print("pull", id)
# 创建机器人实例列表
robot = [Robot() for _ in range(robot_num + 10)]

# 泊位Berth类定义及其声明
class Berth:
    # 初始化泊位的属性：位置、运输时间、装载速度
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0, g_num=0, status=0):
        self.x = x
        self.y = y
        self.transport_time = transport_time # 产生收益的时间 即从泊位到虚拟点的时间
        self.loading_speed = loading_speed # 泊位中的货物装载到船上的时间
        self.g_num = g_num # 泊位中剩余货物数
        self.status = status # 泊位中是否有船舶

# 创建泊位实例列表
berth = [Berth() for _ in range(berth_num + 10)]

# 船舶Boat类定义
class Boat:
    # 初始化船的属性：编号、位置、状态
    def __init__(self, num=0, pos=0, status=0, start_zhen=0, end_zhen=0):
        self.num = num # 装载货物数
        self.pos = pos # 船舶目的泊位ID -1~9 分别代表虚拟点和十个泊位
        self.status = status # 船舶状态
        self.start_zhen = start_zhen
        self.end_zhen = end_zhen

# 尝试在函数内部不修改boat参数 猜测判题器会根据场面信息来输入 
# 尝试失败 代码已回退 可能原因：任务书说明理解不透彻导致更改代码不完善

    # 船舶前往泊位 boat[id]前往berth[pos]
    def ship(self, id, pos):
        self.status = 0 # 船舶由正常运行转为移动中
        print("ship", id, pos)

    # 船舶回虚拟点
    def go(self, id):
        # 泊位状态更新
        berth[self.pos].status = 0
        # 船舶状态更新
        self.status = 0
        self.pos = -1
        print("go", id)

    # 泊位处装载货物 可能需要移植到Berth类中
    def load(self):
        self.status = 1 # 船舶处于装货状态
        self.num = min(self.num + min(berth[self.pos].loading_speed, berth[self.pos].g_num), boat_capacity) # 船舶装载货物数量更新
        berth[self.pos].g_num = min(berth[self.pos].g_num - berth[self.pos].loading_speed, 0) # 船舶所在泊位货物数更新

# 创建船实例列表
boat = [Boat() for _ in range(10)]

# 更多全局变量定义
money = 0  # 货币数量
boat_capacity = 0  # 船的容量
id = 0
ch = []  # 可能用于存储地图信息   可以修改感觉
gds = [[0 for _ in range(N)] for _ in range(N)]  # 二维数组，用于存储货物信息
#ljr 3_7增加了一个存储货物信息的列表
goods=[]
temp = 0 # 储存机器人在哪个泊位

mapp = np.zeros((200, 200), dtype=int) #二维数组 用于存储地图

def read_mapp(i,line):
    for j in range(0,n):
        if line[j] == '*' or line[j] == '#':
            mapp[i][j]=0
        else:
            mapp[i][j]=1

def Init():
    # 初始化函数，用于从输入读取初始配置
    #地图
    for i in range(0, n):
        line = input()
        read_mapp(i,line)
        ch.append(list(line))  #修改为 解析出每一个字符
    #港口信息
    for i in range(berth_num):
        line = input()
        berth_list = [int(c) for c in line.split(sep=" ")]
        id = berth_list[0]
        berth[id].x = berth_list[1]
        berth[id].y = berth_list[2]

        goal_berth.append((berth[id].x,berth[id].y))

        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
    # 船舶的容量
    global boat_capacity # 实验证明需要声明boat_capacity是全局变量 否则会被视为在Init函数内部的局部变量    
    boat_capacity = int(input())
    # 结束阶段
    okk = input()
    print("OK")
    sys.stdout.flush()

def Input():
    # 输入函数，用于在每一"帧"读取新的输入数据并更新系统状态
    id, money = map(int, input().split(" "))
    num = int(input())
    for i in range(num):
        x, y, val = map(int, input().split())

        goods.append(((x,y),zhen)) # 标记货物位置         多出来的参数是计时相关的么？？？？？？？？？

        gds[x][y] = val

    for i in range(robot_num):
        robot[i].goods, robot[i].x, robot[i].y, robot[i].status = map(int, input().split())

    for i in range(5):
        boat[i].status, boat[i].pos = map(int, input().split())
    okk = input()
    return id



if __name__ == "__main__":
    # 主程序入口
    Init()  # 调用初始化函数
    for zhen in range(1, 15001):
        # 主循环，模拟15000帧的运行
        id = Input()  # 处理每帧的输入
        #fmy 3.8 根据交互时的处理顺序 需先输出机器人的指令 再输出船的指令
        # 输出机器人指令
        for i in range(robot_num):
            # 根据机器人可以在同一帧内执行多条指令 先输出move在输出get/pull
            if robot[i].status == 0:
                # 机器人处于恢复状态 无法工作 不输出指令
                pass
            else: # 机器人正常运行
                # 控制每个机器人移动
                move_order = robot[i].move()
                print("move", i, move_order)

                # 机器人坐标更新
                # PS:地图坐标规则: 往下为 X 轴正方向，往右为 Y 轴正方向。
                if move_order == 0: # 右移一格
                    robot[i].y += 1
                elif move_order == 1: # 左移一格
                    robot[i].y -= 1
                elif move_order == 2: # 上移一格
                    robot[i].x -= 1
                else: # 下移一格
                    robot[i].x += 1

                # 如果机器人未装载货物且位于货物生成处 则get取货物
                if robot[i].goods == 0 and gds[robot[i].x][robot[i].y] > 0:
                    robot[i].get(i)

                # 如果机器人装载了货物且处于泊位 则pull放下货物
                # if robot[i].goods == 1 and is_robot_at_any_port(robot[i],berth):
                temp = int_is_robot_at_any_port(robot[i],berth)
                if robot[i].goods == 1 and temp >0 :
                    robot[i].pull(i)
            sys.stdout.flush()

        # 输出船舶指令
        for i in range(0,5):
            """泊位与泊位之间的到达时间为500帧
            fmy 3.8-3.9
            在装载货物方面 船舶可在到达泊位的同一帧内立刻装货 这里可能需要结合泊位的loading_speed来判断
            同时装载货物是泊位的操作 需要结合泊位使用 故先ship在装货

            在运送货物回到虚拟点产生利益方面 需要结合泊位的transport_time来使用 但是感觉暂时用不到
            -----------------------------------------------------------------------------------------
            """
            # 最外部大循环先判断当前船舶状态
            if boat[i].status == 0:
                # 船舶在运输过程中 不输出指令
                # 奇思妙想 在运输过程中船舶是否可以根据泊位的最新情况来中途改道
                pass
            elif boat[i].status == 1:
                # 船舶正常运行 可分为正在在泊位装货或者已经运输完成抵达虚拟点
                if boat[i].pos == -1:
                    # 船舶抵达虚拟点 需要再次出发
                    boat[i].ship(i, random.randint(0, 9)) # 此处需要判断去哪个泊位
                else:
                    # 船舶在泊位 分为正在当前泊位or前往另一泊位
                    # 暂不考虑前往另一泊位 在一棵树上吊死 参考思路见下面注释代码
                    # 即正在装货 可分为当前帧可装满或者不可装满
                    # 当前帧可装满则go 不可装满则更新船舶数据
                    if min(boat[i].num + min(berth[boat[i].pos].loading_speed, berth[boat[i].pos].g_num), boat_capacity) == boat_capacity:
                        # 当前帧可以装满 回虚拟点
                        boat[i].go(i)
                    #------------------------------------------------------------------------------------------------------------
                    # 这段看看分数能否实现零的突破
                    elif min(boat[i].num + min(berth[boat[i].pos].loading_speed, berth[boat[i].pos].g_num), boat_capacity) > 0:
                        # 有货了 直接回去
                        boat[i].go(i)
                    #------------------------------------------------------------------------------------------------------------
                    else:
                        # 当前帧不能装满 继续装
                        boat[i].load()
            else: # boat[i].status == 2
                # 船舶正在泊位处等待 可分为先前位于这个泊位内部的船舶正好在当前帧装满回虚拟点和无法装满执行后续策略(继续等待or去另外的泊位)
                if berth[boat[i].pos].status == 0:
                    # 泊位空出来 程序强行改变等待船舶的状态
                    # PS: 判题器一帧内结算无法实现泊位空出来后立刻让等待船舶进入泊位
                    berth[boat[i].pos].status = 1 # 船舶进入泊位
                    boat[i].load() # 船舶装载货物
                else:
                    # 泊位尚未空闲 暂定策略为等待 不输出指令
                    pass
            # #----------------------------------------------------------------------------------------
            # 分支思路借鉴
            #     else: # 船舶准备前往某一泊位
            #         # 船舶正好位于这一泊位
            #         if boat[i].berth_id == boat[i].pos:
            #             berth[boat[i].berth_id].status = 1
            #             # if boat[i].loaded == boat_capacity:
            #             #     # 装满了 回虚拟点
            #             #     boat[i].go(i)
            #             # else: # 未装满 继续装
            #             #     boat[i].load()
            #         else: # 船舶准备前往其他泊位
            #             boat[i].ship(i, boat[i].pos)
            # #----------------------------------------------------------------------------------------
            sys.stdout.flush()
        print("OK")
        sys.stdout.flush()
