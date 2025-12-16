import time

from ur_rtde import RobotRTDE
from ur_tool import Tool
from config import ROBOT_IP, GRIP_PROGRAM, RELEASE_PROGRAM

robot = RobotRTDE(ROBOT_IP)
tool = Tool(ROBOT_IP)

pose = robot.get_joint_angles()
robot.move("movej", [1.524572491645813, -0.9488840264132996, 1.3860052267657679, -2.0061055622496546, -1.5892236868487757, -0.0950844923602503], v=0.05)

direction = [0.0, 0.0, -1.0]
robot.move_until_contact(direction)

tool.grip(GRIP_PROGRAM)
time.sleep(1)
robot.move("movej", [1.524572491645813, -0.9488840264132996, 1.3860052267657679, -2.0061055622496546, -1.5892236868487757, -0.0950844923602503], v=0.05)
time.sleep(1)
tool.release(RELEASE_PROGRAM)

robot.close()