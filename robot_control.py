from fontTools.misc.cython import returns

from ur_rtde import RobotRTDE
from ur_tool import Tool

class Robot:
    def __init__(self, host):
        self.rtde = RobotRTDE(host)
        self.tool = Tool(host)

    def move(self, move_type, target, a=1.0, v=0.1):
        return self.rtde.move(move_type, target, a, v)

    def move_towards(self, vector, distance, a=0.1, v=0.2):
        return self.rtde.move_towards(vector, distance, a, v)

    def get_tcp_pose(self):
        return self.rtde.get_tcp_pose()

    def get_joint_angles(self):
        return self.rtde.get_joint_angles()

    def move_until_contact(self, direction, force_threshold=30.0, max_time=15.0, a=0.05, v=0.05):
        return self.rtde.move_until_contact(direction, force_threshold, max_time, a, v)

    def move_circle_xy(self, center, radius, steps=100, a=0.5, v=0.05):
        return self.rtde.move_circle_xy(center, radius, steps=steps, a=a, v=v)

    def move_spiral_xy(self, center, turns=3, steps=400, k=0.002, a=0.5, v=0.05):
        return self.rtde.move_spiral_xy(center, turns, steps, k, a, v)

    def move_flower_4_triangles(self, center, side=0.05, a=0.5, v=0.05):
        return self.rtde.move_flower_4_triangles(center, side, a, v)

    def move_triangle(self, side_length=0.005, a=0.3, v=0.02):
        return self.move_triangle(side_length=side_length, a=a, v=v)

    def parse_bits_DI(self, count: int = 18):
        return self.rtde.parse_bits(count=count)

    def grip(self, name_file):
        self.tool.grip(name_file)
        self.rtde.reconnect()

    def release(self, name_file):
        self.tool.release(name_file)
        self.rtde.reconnect()

    def close(self):
        self.rtde.close()
        self.tool.close()