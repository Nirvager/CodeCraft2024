
import sys
import random
# 定义全局变量
n = 200  # 地图的尺寸或某个特定的数量标志
robot_num = 10  # 机器人的数量
berth_num = 10  # 泊位的数量
N = 210  # 用于创建二维数组时的尺寸，略大于n，可能是为了处理边界情况

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
    #ljr 3.7  15:50尝试思路1：尽量往一个港口靠
    def move(self,direction):
        delta_x = self.x-berth[0].x
        delta_y = self.y-berth[0].y


        # 根据差异的绝对值比较，确定主要的移动方向
        if abs(delta_x) > abs(delta_y) :
            # 如果x方向的差异大于y方向的，优先左右移动
            return 0 if delta_x > 0 else 1
        else:
            # 否则优先上下移动
            return 2 if delta_y > 0 else 3



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
        print("ship",0,1)
        for i in range(robot_num):
            # 控制每个机器人随机移动
            print("move", i, robot[i].move())
            print("get", i)
            print("pull",i)
            sys.stdout.flush()
        print("OK")
        sys.stdout.flush()
