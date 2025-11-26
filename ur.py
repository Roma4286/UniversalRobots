import socket
import time
import math
import struct

robotIP = "10.162.3.228"
PRIMARY_PORT = 30001
SECONDARY_PORT = 30002
REALTIME_PORT = 30003

base_point = f"[0, -1.57, 0, -1.57, 0, 0]"

class Robot:
    def __init__(self, ip, primary_port = 30001, secondary_port = 30002, realtime_port = 30003, port_now = 30001):
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
            # f = self.get_tcp_speed()
            state = self.get_runtime_state()
            # print(f)

    def send_urscript(self, cmd: str):
        self.s.sendall((cmd + "\n").encode("utf-8"))
        time.sleep(0.05)

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

    def get_tcp_force(self):
        data = self.get_tcp_data(4096)
        if len(data) < 48:
            return None

        wrench = struct.unpack("!6d", data[-48:])
        return wrench  # Fx, Fy, Fz, Tx, Ty, Tz

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
        # Вычисляем новую позицию
        new_v = [
            ve * vector[0],
            ve * vector[1],
            ve * vector[2]
        ]
        return new_v

    def move_until_contact(self, direction, a=0.1, v=0.05, speed_threshold=0.003):

        norm_dir = math.sqrt(direction[0] ** 2+direction[1] ** 2+direction[2] ** 2)
        if norm_dir == 0:
            raise ValueError("Вектор направления не может быть нулевым")
        unit_dir = [d / norm_dir for d in direction]

        self.send_urscript(f"speedl([0, 0, 0.05, 0, 0, 0], a={a}, t=3)")
        # self.move_towards(direction, 50, a, v)

        while True:
            tcp_speed = self.get_tcp_speed()
            if tcp_speed is None:
                continue

            lin_speed_norm = math.sqrt(tcp_speed[0] ** 2+tcp_speed[1] ** 2+tcp_speed[2] ** 2)
            if lin_speed_norm < speed_threshold:
                self.send_urscript("stopl(10.0)")
                print("Контакт достигнут, движение остановлено")
                break

            time.sleep(0.05)


robot = Robot(robotIP)
robot.send_urscript("zero_ftsensor()")
f = (robot.get_tcp_speed())
print(f"1+{f}")
robot.move("movej", "[1.5399072170257568, -0.2457065147212525, 1.2899506727801722, -2.6184002361693324, -1.5765298048602503, -0.11803323427309209]", 0.05, 0.3)
# robot.send_urscript(f"movej([1.5399072170257568, -0.2457065147212525, 1.2899506727801722, -2.6184002361693324, -1.5765298048602503, -0.11803323427309209], a=0.05, v=0.3)")

# Текущая TCP позиция робота (можно получить через RTDE/30003 или сохранить)
current_pose = [.148176530408, -.770704002574, -.312483911963, .131255194183, -3.134044456313, .004624001678]
# robot.send_urscript(f"movel(p[.148176530408, -.770704002574, -.012483911963, .131255194183, -3.134044456313, .004624001678], a=0.05, v=0.1)")
d, v = robot.get_tcp_pose(), robot.move_kirill([0.0, 0.0, 1.0], 0.15)
print(d, v)
robot.send_urscript('textmsg("11111111111")')

urscript = f"""
force_mode({d}, 
           {[0,0,1,0,0,0]},
           {[0,0,10,0,0,0]},
           2,
           [{0.05}, {0.05}, {0.05}, 0.1, 0.1, 0.1])
           
sleep(0.5)
end_force_mode()
"""
robot.send_urscript(f"force_mode(p{d},[0,0,-1,0,0,0],[0,0,50,0,0,0],2,[1,1,0.1,d2r(10),d2r(10),1])\n"
                    f"sleep(0.5)\n"
                    f"end_force_mode()\n")
robot.send_urscript('textmsg("222222222222222")')
robot.move_until_contact([0, 0, -1], a=0.1, v=0.05, speed_threshold=0.0007)

# # Направление к объекту
# direction = [0.0, 0.0, 1.0]  # движение вдоль оси Z
#
# # Расстояние движения
# distance = 0.7  # 100 мм
#
# # Двигаем
# robot.move_towards(direction, distance, a=1.0, v=0.1)
d = robot.get_tcp_pose()
print(d)
# robot.send_urscript("movej([0, -1.57, 0, -1.57, 0, 0], a=0.1, v=0.4)")
robot.move("movej", base_point, 0.1, 0.4)

robot.close_connect()
