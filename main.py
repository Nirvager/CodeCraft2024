
import sys
import random
# 定义全局变量
n = 200  # 地图的尺寸或某个特定的数量标志
robot_num = 10  # 机器人的数量
berth_num = 10  # 泊位的数量
N = 210  # 用于创建二维数组时的尺寸，略大于n，可能是为了处理边界情况

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

#使用一个简单的A*启发式搜索
def get_feasible_directions(current_pos):
    """获取没有被障碍物阻塞的方向"""
    directions = [2, 3, 1, 0] #分别对应 上下左右  #图中左上角为原点 向下为x正 向右为y正
    dx_dy=[(-1,0),(1,0),(0,-1),(0,1)]
    feasible_directions = []

    #zip是一个合成元组的东西
    for direction,(dx,dy) in zip(directions,dx_dy):
        next_pos = (current_pos[0]+dx,current_pos[1]+dy)
        #检查下一步是否被障碍物阻塞 can be optimized in the future
        if ch[next_pos[0]][next_pos[1]] == '.':
            feasible_directions.append((direction,next_pos))

    return feasible_directions


def get_best_direction(current_pos,goal_pos):
    """基于当前位置、目标位置和障碍物，决定最优的单步移动方向"""
    feasible_directions = get_feasible_directions(current_pos)

    #计算每个可行方向的启发式成本
    best_direction = feasible_directions[0][0]
    min_cost = float('inf')
    for direction,next_pos in feasible_directions:
        cost = abs(next_pos[0]-goal_pos[0]) + abs(next_pos[1]-goal_pos[1])
        if cost < min_cost:
            min_cost=cost
            best_direction=direction

    return best_direction


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

    #move 机器人的移动策略
    #ljr 3.7  晚上 尝试思路1：A*
    def move(self):
        #先判断是否装了货物
        #已经装了货物
        if self.status == 1:  ###一开始货物很少 我们让机器人直接去码头
            nearest_berth = find_nearestberth((self.x,self.y),berth)
            direction=get_best_direction((self.x,self.y),(nearest_berth.x,nearest_berth.y))
        #未安装货物
        if self.status == 0:
            nearest_goods = find_nearestgoods((self.x,self.y),goods)
            direction = get_best_direction((self.x, self.y), nearest_goods)
        return direction



# 创建机器人实例列表
robot = [Robot() for _ in range(robot_num + 10)]

class Berth:
    # 初始化泊位的属性：位置、运输时间、装载速度
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0):
        self.x = x
        self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed


# 创建泊位实例列表
berth = [Berth() for _ in range(berth_num + 10)]

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
#boat_capacity = 0  # 船的容量
boat_capacity=[int for _ in range(10)]
id = 0
ch = []  # 可能用于存储地图信息   可以修改感觉
gds = [[0 for _ in range(N)] for _ in range(N)]  # 二维数组，用于存储货物信息
#ljr 3_7增加了一个存储货物信息的列表
goods=[]

def Init():
    # 初始化函数，用于从输入读取初始配置
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

        goods.append((x,y))

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
        for i in range(robot_num):
            # 控制每个机器人随机移动
            print("move", i, robot[i].move())
            print("get", i)
            print("pull",i)
            sys.stdout.flush()
        print("OK")
        sys.stdout.flush()
