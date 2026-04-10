import math
import time
from config import ROBOT_IP, GRIP_PROGRAM, RELEASE_PROGRAM
from mouse_hold import mouse_pressing, mouse_releasing
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
    final_center = [-0.1897170618094012, -0.7351738542729902, -0.008215309818223326, 0.36048330742355195, -3.0940601727750954, 0.0018451418590662097]
    print(pose)
    center = [final_center[0], final_center[1]]

    try:
        robot.move("movel", final_center, a=0.01, v=0.01)
        mouse_pressing()
        robot.move_matching_spiral(center, star_radius=0.03, v = 0.01)
        # robot.move_4flower_triangles(center, side=0.02, v=0.01)
        # robot.move_6flower_triangles(center, side=0.02, v=0.01)
        # robot.move_flower_triangles(center, side=0.02, v=0.01)
        # robot.move_triangle(0.005, v=0.5)
        # robot.move_circle_xy(center, radius=0.05, v=0.05)

        # robot.move_5point_star(n=8,radius=0.01, v=0.01)
        mouse_releasing()

    finally:
        robot.close()