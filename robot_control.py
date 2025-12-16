from ur_rtde import RobotRTDE
from ur_tool import Tool

class Robot:
    def __init__(self, host):
        self.rtde = RobotRTDE(host)
        self.tool = Tool(host)

    # Методы движения
    def move(self, move_type, target, a=1.0, v=0.1):
        return self.rtde.move(move_type, target, a, v)

    def get_tcp_pose(self):
        return self.rtde.get_tcp_pose()

    def get_joint_angles(self):
        return self.rtde.get_joint_angles()

    def move_until_contact(self, direction, force_threshold=30.0, max_time=10.0, a=0.05, v=0.05):
        return self.rtde.move_until_contact(direction, force_threshold, max_time, a, v)

    # Методы работы с инструментом
    def grip(self, name_file):
        self.tool.grip(name_file)
        self.rtde.reconnect()

    def release(self, name_file):
        self.tool.release(name_file)
        self.rtde.reconnect()

    # Закрытие
    def close(self):
        self.rtde.close()
        self.tool.close()