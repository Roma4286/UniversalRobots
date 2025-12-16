import socket
import time
import math
import struct

from config import ROBOT_IP, PRIMARY_PORT, SECONDARY_PORT, REALTIME_PORT, BASE_POINT

class Robot:
    def __init__(self, ip, primary_port = PRIMARY_PORT, secondary_port = SECONDARY_PORT, realtime_port = REALTIME_PORT, port_now = PRIMARY_PORT):
        self.ip = ip
        self.primary_port = primary_port
        self.secondary_port = secondary_port
        self.realtime_port = realtime_port
        self.port = port_now

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.ip, self.port))

    # @classmethod
    # def degrees_to_radians(cls, corner: int):
    #     return corner * pi / 180

    def get_tcp_data(self, l):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2)
        try:
            s.connect((self.ip, self.realtime_port))
            data = s.recv(l)
            return data
        finally:
            s.close()

    def wait_end_of_move(self):
        state = self.get_runtime_state()

        while state != 0:
            time.sleep(0.05)
            f = self.get_tcp_force()
            state = self.get_runtime_state()
            # print(f)

    def send_urscript(self, cmd: str):
        d = self.s.sendall((cmd + "\n").encode("utf-8"))
        time.sleep(0.5)
        return d

    def move(self, type_of_move: str, final_pose: str, a: float, v: float):
        self.send_urscript(f"{type_of_move}({final_pose}, a={a}, v={v})")

        self.wait_end_of_move()

    def move_towards(self, vector, distance, a=0.1, v=0.2):
        current_pose = self.get_tcp_pose()
        #Нормируем вектор направления
        al = math.sqrt(distance**2/(vector[0]**2+vector[1]**2+vector[2]**2))
        if al == 0:
            raise None
        new_pose = [
            current_pose[0]+vector[0]*al,
            current_pose[1]+vector[1]*al,
            current_pose[2]+vector[2]*al,
            current_pose[3],
            current_pose[4],
            current_pose[5]
        ]

        self.move("movel", f"p{new_pose}", a, v)

    def move_until_contact(self, vector, force_threshold=30.0, a=0.05, v=0.05):
        """
        Движение вдоль direction-vector до момента контакта.
        Контакт определяется по силе в TCP force (Fx, Fy, Fz).
        """

        # нормируем вектор направления
        norm = math.sqrt(sum(c*c for c in vector))
        if norm == 0:
            raise ValueError("Vector must not be zero")

        tcp_v = self.move_kirill(vector, v)

        # запускаем движение с постоянной скоростью
        cmd = f"speedl([{tcp_v[0]}, {tcp_v[1]}, {tcp_v[2]}, 0, 0, 0], a={a}, t=10)"
        self.send_urscript(cmd)

        print("Начинаю движение до контакта...")

        # читаем силы каждые 20–30 мс
        while True:
            Fx, Fy, Fz, Tx, Ty, Tz = self.get_tcp_force()

            # модуль силы
            force = math.sqrt(Fx**2 + Fy**2 + Fz**2)

            print(f"Force = {force:.3f}  (Fx={Fx:.2f}, Fy={Fy:.2f}, Fz={Fz:.2f})")

            # Условие контакта — сила превысила порог
            if force > force_threshold:
                print("Контакт обнаружен!")
                break

            time.sleep(0.02)

        # Мягкая остановка
        self.send_urscript("stopl(0.2)")



    def get_tcp_force(self):
        data = self.get_tcp_data(1108)
        if len(data) < 48:
            return None

        tcp_force = struct.unpack('!6d', data[540:540+48])

        print("TCP forces:", tcp_force)
        return tcp_force # Fx, Fy, Fz, Tx, Ty, Tz


    def get_tcp_speed(self):
        data = self.get_tcp_data(1116)
        if len(data) < 540:
            return None

        speed_offset = 492
        tcp_speed = [
            struct.unpack('!d', data[speed_offset+i * 8: speed_offset+(i+1) * 8])[0]
            for i in range(3)
        ]
        return tcp_speed

    def get_runtime_state(self):
        data = self.get_tcp_data(1116)
        if len(data) < 1116:
            return None
        robot_mode_offset = 86
        runtime_state_offset = robot_mode_offset + 24
        return data[runtime_state_offset]


    def get_tcp_pose(self):
        data = self.get_tcp_data(1116)
        if len(data) < 1116:
            return None

        pose_offset = 444
        pose = [
            struct.unpack('!d', data[pose_offset+i * 8: pose_offset+(i+1) * 8])[0]
            for i in range(6)
        ]
        return pose

    def close_connect(self):
        self.s.close()


    def move_kirill(self, vector, v):
        #Нормируем вектор направления
        ve = math.sqrt(v**2/(vector[0]**2+vector[1]**2+vector[2]**2))
        print(ve)
        if ve == 0:
            raise None

        new_v = [
            ve * vector[0],
            ve * vector[1],
            ve * vector[2]
        ]
        return new_v

robot = Robot(ROBOT_IP, port_now=SECONDARY_PORT)
time.sleep(1)
robot.move("movej", f"{BASE_POINT}", 0.2, 0.2)
# time.sleep(10)
# with open("Robot/Robot1.script", "r", encoding="utf-8") as f:
#     urscript_code = f.read()

# robot.send_urscript(urscript_code)
time.sleep(5)
robot.close_connect()
