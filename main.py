import time

from config import ROBOT_IP, GRIP_PROGRAM, RELEASE_PROGRAM
from robot_control import Robot

robot = Robot(ROBOT_IP)

pose = robot.get_joint_angles()
robot.move("movej", [1.524572491645813, -0.9488840264132996, 1.3860052267657679, -2.0061055622496546, -1.5892236868487757, -0.0950844923602503], v=0.05)

direction = [0.0, 0.0, -1.0]
robot.move_until_contact(direction)

robot.grip(GRIP_PROGRAM)
time.sleep(1)
robot.move("movej", [1.524572491645813, -0.9488840264132996, 1.3860052267657679, -2.0061055622496546, -1.5892236868487757, -0.0950844923602503], v=0.05)
time.sleep(1)
robot.release(RELEASE_PROGRAM)

robot.close()