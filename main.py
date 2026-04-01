import math
import time
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

if __name__ == "__main__":
    robot = Robot(ROBOT_IP)
    pose = robot.get_tcp_pose()
    time.sleep(3)
    center = [pose[0], pose[1]]
    print(center)
    try:
        # robot.move_spiral_xy(center, v=0.05)
        # robot.move_4flower_triangles(center, side=0.02, v=0.01)
        # robot.move_6flower_triangles(center, side=0.02, v=0.01)
        # robot.move_8flower_triangles(center, side=0.02, v=0.01)
        # robot.move_triangle(0.005, v=0.5)
        robot.move_5point_star(center)
        # robot.move_circle_xy(center, radius=0.05, v=0.05)
    finally:
        robot.close()