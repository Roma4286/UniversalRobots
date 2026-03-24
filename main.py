import math
import time
from idlelib.configdialog import tracers
from os import times

from config import ROBOT_IP, GRIP_PROGRAM, RELEASE_PROGRAM
from robot_control import Robot

# robot = Robot(ROBOT_IP)

# pose = robot.get_tcp_pose()

# time.sleep(4)
# center = [pose[0], pose[1]]
# print(center)
# # center = [-0.22607741205821605, -0.7990163895167282]
# radius = 0.01               # 10 см
#
# robot.move_circle_xy([-0.18603362735130755, -0.7990192814183934], radius, steps=200)
# time.sleep(1)
#
# robot.close()

import math
import time

def move_triangle(robot: Robot, side_length=0.005, a=0.3, v=0.02):
    """
    Движение по равностороннему треугольнику в плоскости XY

    side_length — длина стороны (в метрах, 0.05 = 5 см)
    """
    time.sleep(4)

    # Текущая позиция TCP
    start_pose = robot.get_tcp_pose()  # [x, y, z, rx, ry, rz]

    x0, y0, z0, rx, ry, rz = start_pose

    # Вершины равностороннего треугольника
    p1 = [x0, y0, z0, rx, ry, rz]
    p2 = [x0+side_length, y0, z0, rx, ry, rz]

    # третья точка через геометрию (60 градусов)
    height = side_length * math.sqrt(3) / 2
    p3 = [x0+side_length / 2, y0+height, z0, rx, ry, rz]

    # Движение по точкам
    robot.move("movel", p2, a, v)
    robot.move("movel", p3, a, v)
    robot.move("movel", p1, a, v)


if __name__ == "__main__":
    robot = Robot(ROBOT_IP)  # IP робота
    pose = robot.get_tcp_pose()
    time.sleep(3)
    center = [pose[0], pose[1]]
    print(center)
    try:
        # robot.move_spiral_xy(center, v=0.05)
        robot.move_flower_4_triangles(center, side=0.02, v=0.01)
    finally:
        robot.close()