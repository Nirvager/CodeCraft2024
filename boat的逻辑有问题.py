import sys
import random
# 定义全局变量
n = 200  # 地图的尺寸或某个特定的数量标志
robot_num = 10  # 机器人的数量
berth_num = 10  # 泊位的数量
N = 210  # 用于创建二维数组时的尺寸，略大于n，可能是为了处理边界情况

# #ljr3_8 尝试追踪一下每个点的运动
# move_history={}

# def update_move_history(current_pos, direction):
#     if current_pos not in move_history:
#         move_history[current_pos] = []
#     move_history[current_pos].append(direction)

# def is_repetitive_movement(current_pos, direction):
#     # 检查当前位置是否有记录
#     if current_pos not in move_history:
#         return False
#     history = move_history[current_pos]
#     # 如果历史记录中包含连续的来回移动，则认为是反复运动
#     if len(history) >= 2 and history[-1] == (history[-2]) and history[-2]==direction:
#         return True
#     return False
def is_repetitive_movement(move_history, choice):
    """检查重复模式"""
    if len(move_history) <=3:
        return False
    if choice[1] == opposite_direction(move_history[-1][1]):
        return True
    return False

def opposite_direction(direction):
    opposites = {2:3,3:2,1:0,0:1}
    return opposites.get(direction)

#ljr定义一下曼哈顿距离 后续可修改
def distance(a, b) :
    return abs(a[0]-b[0])+abs(a[1]-b[1])

#找最近的货物
def find_nearestgoods(robot_pos, goods):
    nearest_goods = min(goods, key=lambda goods_pos:abs(robot_pos[0]-goods_pos[0])+abs(robot_pos[1])-goods_pos[1])
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

def is_circulate_movement(move_history,choice):
    if choice in move_history:
        return True
    return False

def heuristic(next_pos,goal_pos):
    base_cost = abs(next_pos[0] - goal_pos[0]) + abs(next_pos[1] - goal_pos[1])
    # 如果已经很接近目标，可能不需要调整成本
    if base_cost < 100:
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

# def get_best_direction(current_pos,goal_pos):
#     """基于当前位置、目标位置和障碍物，决定最优的单步移动方向"""
#     feasible_directions = get_feasible_directions(current_pos)

#     #计算每个可行方向的启发式成本
#     best_direction = 0
#     min_cost = float('inf')
#     for direction,next_pos in feasible_directions:
#         cost = abs(next_pos[0]-goal_pos[0]) + abs(next_pos[1]-goal_pos[1])
#         if cost < min_cost:
#             min_cost=cost
#             best_direction=direction

#     update_move_history(current_pos,best_direction)

#     return best_direction
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

#ljr3.8判断一下机器人是否在港口
def is_robot_at_any_port(robot, berth):
    for port in berth:
        # 每个泊位具有实际面积4x4，而非1x1，所以应当判断处于一个范围内
        # if robot.x == port.x and robot.y == port.y:
        if port.x <= robot.x <= (port.x + 3) and port.y <= robot.y <= (port.y + 3):
            return True
    return False

# 机器人Robot类定义及其功能函数
class Robot:
    # 初始化机器人的属性：位置、携带的货物、状态、目标泊位的坐标
    def __init__(self, startX=0, startY=0, goods=0, status=0, mbx=0, mby=0):
        self.x = startX # 初始X坐标 上下方向
        self.y = startY # 初始Y坐标 左右方向
        self.goods = goods # 是否携带货物
        self.status = status # 机器人是否处于撞击或沉海后的恢复过程中
        self.mbx = mbx
        self.mby = mby
        self.move_history=[]

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
        if self.goods == 1:  ###一开始货物很少 我们让机器人直接去码头
            nearest_berth = find_nearestberth((self.x,self.y),berth)
            direction=get_best_direction((self.x,self.y),(nearest_berth.x,nearest_berth.y),self.move_history)
            self.move_history.append(((self.x,self.y),direction))
        #未安装货物
        if self.goods == 0:
            nearest_goods = find_nearestgoods((self.x,self.y),goods)
            direction = get_best_direction((self.x, self.y), nearest_goods,self.move_history)
            self.move_history.append(((self.x,self.y),direction))

        if len(self.move_history) >=180 :
            self.move_history.pop(0)

        return direction
    
    def get(self, id):
        print("get", id)

    def pull(self, id):
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
    def __init__(self, num=0, pos=0, status=0, loaded=0, start_zhen=0, end_zhen=0, berth_id=0):
        self.num = num # 可能是泊位编号 可能是装载货物数 可能是装载货物价值总数
        self.pos = pos # 船舶目的地ID -1~9 分别代表虚拟点和十个泊位
        self.status = status # 船舶状态
        self.loaded = loaded # 装载货物件数
        self.start_zhen = start_zhen
        self.end_zhen = end_zhen
        self.berth_id = berth_id # 所在的泊位ID

    # 船舶开始航行
    def ship(self, id, berth_id):
        self.status = 0 # 状态更新
        print("ship", id, berth_id)

    # 船舶回虚拟点
    def go(self, id):
        # 船舶状态更新
        self.status = 0
        self.pos = -1
        # 泊位状态更新
        berth[self.berth_id].status = 0
        self.berth_id = -1
        print("go", id)

    # 泊位处装载货物
    def load(self):
        self.status = 1 # 状态更新
        # 当前帧结束时船舶装载状态
        self.loaded = min(self.loaded + min(berth[self.berth_id].loading_speed, berth[self.berth_id].g_num), boat_capacity)
        # 当前帧结束时泊位货物剩余数
        berth[self.berth_id].g_num = min(berth[self.berth_id].g_num - berth[self.berth_id].loading_speed, 0)

    # 虚拟点处再次出发
    def restart(self, id, berth_id): # 此处berth_id无法确定
        self.status = 0 # 状态更新
        print("ship", id, berth_id)
# 创建船实例列表
boat = [Boat() for _ in range(10)]

# 更多全局变量定义
money = 0  # 货币数量
boat_capacity = 0  # 船的容量
id = 0
ch = []  # 可能用于存储地图信息
gds = [[0 for _ in range(N)] for _ in range(N)]  # 二维数组，用于存储货物信息
#ljr 3_7增加了一个存储货物信息的列表
goods=[]

def Init():
    # 初始化函数，用于从输入读取初始配置
    global boat_capacity # 修改全局变量
    #地图
    for i in range(0, n):
        line = input()
        ch.append(list(line))  #修改为 解析出每一个字符
    #港口信息
    for i in range(berth_num):
        line = input()
        berth_list = [int(c) for c in line.split(sep=" ")]
        id = berth_list[0]
        berth[id].x = berth_list[1]
        berth[id].y = berth_list[2]
        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
        berth[id].berth_id = id
    boat_capacity = int(input())
    okk = input()
    print("OK")
    sys.stdout.flush()

def Input():
    # 输入函数，用于在每一"帧"读取新的输入数据并更新系统状态
    id, money = map(int, input().split(" "))
    num = int(input())
    for i in range(num):
        x, y, val = map(int, input().split())

        goods.append((x,y)) # 标记货物位置
        # 缺少去除货物位置功能
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
        #fmy 3.8 需先输出机器人的指令 再输出船的指令
        # 输出机器人指令
        for i in range(robot_num):
            # 根据机器人可以在同一帧内执行多条指令 先输出move在输出get/pull

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
            if robot[i].goods == 1 and is_robot_at_any_port(robot[i],berth):
                robot[i].pull(i)

            sys.stdout.flush()
            
        # 输出船舶指令
        for i in range(0,5):
            """泊位与泊位之间的到达时间为500帧
            fmy 3.8 
            在装载货物方面 船舶可在到达泊位的同一帧内立刻装货 这里可能需要结合泊位的loading_speed来判断
            同时装载货物是泊位的操作 需要结合泊位使用 故先ship在装货

            在运送货物回到虚拟点产生利益方面 需要结合泊位的transport_time来使用
            """#------------------------------------------------------------------------------------------
            # 初始情况: 船从虚拟点出发 但是因为没有船的策略 所以随便分配的
            # if zhen % 3000 == 0:
            #     print("ship", zhen // 3000, zhen // 3000)
            # 进阶情况 根据判题器给出的五艘船舶的状态来判断
            if boat[i].status == 0:
                # 船舶移动中 不能操作
                pass
            elif boat[i].status == 2:
                # 船舶正在泊位外等待
                # 未知处理方法 暂定等待泊位为空
                pass
                # if berth[boat[i].berth_id].status :
                #     # 泊位内船舶仍然未走
                #     pass
                # else: # 泊位内船舶已走
                #     berth[boat[i].berth_id].status = 1
                #     boat[i].load()
            else:
                # 船舶正常运行 即位于泊位装货/虚拟点交易完毕等待下一次开始航行
                if boat[i].pos == -1:
                    if boat[i].berth_id == -1:
                        # 船舶已经在虚拟点了
                        boat[i].ship(i, random.randint(1, 9)) # 不知道选择哪个泊位
                    else:
                        # 船舶准备回虚拟点
                        boat[i].go(i)
                    # boat[i].ship(i, boat[i].pos)
                else: # 船舶准备前往某一泊位
                    # 船舶正好位于这一泊位
                    if boat[i].berth_id == boat[i].pos:
                        berth[boat[i].berth_id].status = 1
                        #--------------------------------------------
                        if boat[i].loaded >= 0:
                            boat[i].go(i)
                        #--------------------------------------------
                        # 上面这段是先搞点分用的 后续可删
                        # 下面这段不能删
                        #--------------------------------------------
                        # if boat[i].loaded == boat_capacity:
                        #     # 装满了 回虚拟点
                        #     boat[i].go(i)
                        # else: # 未装满 继续装
                        #     boat[i].load()
                        #---------------------------------------------
                    else: # 船舶准备前往其他泊位
                        boat[i].ship(i, boat[i].pos)
                    
            # 失败的进阶情况: 帧数为1时从虚拟点出发 
            # if zhen == 1: # 开局 先让船舶全部出发 去往五个泊位 具体哪五个是个难题 暂定berth[0-4]
            #     # boat[i].start_zhen = zhen # zhen == 1 出发帧
            #     # boat[i].end_zhen = boat[i].start_zhen + berth[i].transport_time # 出发帧+泊位运输时间=到达帧
            #     # boat[i].berth_id = i
            #     # print("ship", i, i)
            #     pass
            # else: # 局中
            #     if boat[i].status == 0: # boat[i].start_zhen <= zhen < boat[i].end_zhen:
            #         # 此时 该船舶正在加速赶来的路上 
            #         # 到达泊位时可以立即装货 到达虚拟点产生价值时可以立即出发 故 < end_zhen 而非 <= end_zhen
            #         pass
            #     elif boat[i].status == 2:
            #         # 此时 船舶正在泊位外焦急地等待
            #         pass
            #     else: # 此时 船舶可以被操作
                    
            #         # 更新船舶的泊位ID功能实现位置选择
            #         # 到达某泊位时 or 从某泊位出发时
            #         if zhen == boat[i].end_zhen:
            #             # 此时 船舶到达某泊位 需要更新船舶的泊位ID
            #             # 会影响到后续船舶回到虚拟点时计时器的运行
            #             pass

            #         # Boat类新增loaded属性 表示已经装了多少货物
            #         if boat[i].loaded == boat_capacity: # zhei艘船满了 回到虚拟点
            #             # 更新计时器
            #             boat[i].start_zhen = zhen # 当前帧出发  下面这个berth_id没有更新 更新功能尚未实现
            #             boat[i].end_zhen = boat[i].start_zhen + berth[boat[i].berth_id].transport_time
            #             print("go", i)
            #         else: # 船未满 此时船停靠在某个泊位
            #             if 1: # 泊位内有货 泊位装载货物到船舶上 且不能超出其最大装载量
            #                 boat[i].loaded = min(boat[i].loaded+berth[boat[i].berth_id].loading_speed, boat_capacity)
            #             else: # 泊位内没有货
            #                 # 去有货的泊位/等一会?
            #                 # 直接回到虚拟点 先赚一点
            #                 boat[i].start_zhen = zhen # 当前帧出发  下面这个berth_id没有更新 更新功能尚未实现
            #                 boat[i].end_zhen = boat[i].start_zhen + berth[boat[i].berth_id].transport_time
            #                 print("go", i)

            sys.stdout.flush()

        print("OK")
        sys.stdout.flush()
