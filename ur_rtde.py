import time
import math

try:
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface
    from rtde_io import RTDEIOInterface
except Exception as e:
    raise ImportError("Не удалось импортировать RTDE-клиент. Установи rtde/ur-rtde. Ошибка: " + str(e))

from config import SECONDARY_PORT, RTDE_RECEIVE_PORT

class RobotRTDE:
    def __init__(self, host, port_control=SECONDARY_PORT, port_receive=RTDE_RECEIVE_PORT, timeout=5.0):
        self.host = host
        self.timeout = timeout

        self.rtde_r = RTDEReceiveInterface(self.host, port_receive)
        self.rtde_c = RTDEControlInterface(self.host, port_control)
        self.rtde_o = RTDEIOInterface(self.host)

        time.sleep(0.1)

        if not self.is_connected():
            raise RuntimeError("Не удалось подключиться к RTDE интерфейсу на " + self.host)

    def reconnect(self, retries=3, delay=1.0):
        for attempt in range(1, retries+1):
            try:
                self.close()

                self.rtde_r = RTDEReceiveInterface(self.host, RTDE_RECEIVE_PORT)
                self.rtde_c = RTDEControlInterface(self.host, SECONDARY_PORT)
                self.rtde_o = RTDEIOInterface(self.host)
                time.sleep(0.1)

                if self.is_connected():
                    print("[Reconnect] Подключение успешно")
                    return True
                else:
                    print("[Reconnect] Подключение не удалось")
            except Exception as e:
                print(f"[Reconnect] Ошибка: {e}")

            time.sleep(delay)

        raise RuntimeError(f"Не удалось подключиться к RTDE интерфейсу на {self.host} после {retries} попыток")

    def move_until_contact1(self):
        xd = [0, 0, -0.05, 0, 0, 0]
        direction = [0, 0, -1, 0, 0, 0]

        return self.rtde_c.moveUntilContact(xd, direction=direction, acceleration=0.5)

    def is_connected(self):
        try:
            _ = self.rtde_r.getActualTCPPose()
            return True
        except Exception:
            return False

    def is_running(self):
        try:
            return self.rtde_c.isProgramRunning()
        except AttributeError:
            return None

    def get_tcp_pose(self):
        """Возвращает list [x,y,z,rx,ry,rz] или None"""
        try:
            return self.rtde_r.getActualTCPPose()
        except Exception:
            return None

    def get_tcp_force(self):
        """Возвращает list [Fx,Fy,Fz,Tx,Ty,Tz] или None"""
        try:
            return self.rtde_r.getActualTCPForce()
        except Exception:
            return None

    def get_joint_angles(self):
        try:
            return self.rtde_r.getActualQ()
        except Exception:
            return None

    def move(self, move_type: str, target, a=1.0, v=0.1):
        if move_type.lower() == "movej":
            self.rtde_c.moveJ(target, speed=v, acceleration=a)
            return self.stop(a=0.2)
        elif move_type.lower() == "movel" or move_type.lower() == "movel":
            return self.rtde_c.moveL(target, speed=v, acceleration=a)
        else:
            raise ValueError("Unsupported move type: " + move_type)

    def move_spiral_xy(self, center, turns=3, steps=400, k=0.002, a=0.5, v=0.05):
        pose = self.get_tcp_pose()

        z = pose[2]
        rx, ry, rz = pose[3:]

        cx, cy = center

        path = []

        for i in range(steps):
            theta = 2 * math.pi * turns * i / steps

            r = k * theta

            x = cx+r * math.cos(theta)
            y = cy+r * math.sin(theta)

            path.append([x, y, z, rx, ry, rz, v, a, 0.002])

        theta = 2 * math.pi * turns
        r = k * theta
        x = cx+r * math.cos(theta)
        y = cy+r * math.sin(theta)

        path.append([x, y, z, rx, ry, rz, v, a, 0.0])

        self.rtde_c.moveL(path)

        self.rtde_c.moveL([x, y, z, rx, ry, rz], v, a)

    def move_4flower_triangles(self, center, side=0.05, a=0.5, v=0.05):

        pose = self.get_tcp_pose()
        z = pose[2]
        rx, ry, rz = pose[3:]

        cx, cy = center

        path = []
        h = side*(math.sqrt(3)/2)
        angles = [0, math.pi/2, math.pi, math.pi*3/2]

        for angle in angles:
            dx = math.cos(angle)
            dy = math.sin(angle)

            px = -dy
            py = dx

            p1 = [cx, cy]  # центр
            p2 = [cx+dx * h+px * side / 2, cy+dy * h+py * side / 2]
            p3 = [cx+dx * h-px * side / 2, cy+dy * h-py * side / 2]

            path.append([p1[0], p1[1], z, rx, ry, rz, v, a, 0.002])
            path.append([p2[0], p2[1], z, rx, ry, rz, v, a, 0.002])
            path.append([p3[0], p3[1], z, rx, ry, rz, v, a, 0.002])
            path.append([p1[0], p1[1], z, rx, ry, rz, v, a, 0.002])

        path[-1][-1] = 0.0

        self.rtde_c.moveL(path)
    def move_6flower_triangles(self, center, side=0.05, a=0.5, v=0.05):

        pose = self.get_tcp_pose()
        z = pose[2]
        rx, ry, rz = pose[3:]

        cx, cy = center

        path = []
        half_base = (side*0.6)/2
        h = math.sqrt(pow(side, 2) - pow(half_base, 2))
        degree_angle = math.pi/3
        angles = [0, degree_angle, degree_angle*2, degree_angle*3, degree_angle*4, degree_angle*5]

        for angle in angles:
            dx = math.cos(angle)
            dy = math.sin(angle)

            px = -dy
            py = dx

            p1 = [cx, cy]  # центр
            p2 = [cx+dx * h+px * side / 2, cy+dy * h+py * side / 2]
            p3 = [cx+dx * h-px * side / 2, cy+dy * h-py * side / 2]

            path.append([p1[0], p1[1], z, rx, ry, rz, v, a, 0.002])
            path.append([p2[0], p2[1], z, rx, ry, rz, v, a, 0.002])
            path.append([p3[0], p3[1], z, rx, ry, rz, v, a, 0.002])
            path.append([p1[0], p1[1], z, rx, ry, rz, v, a, 0.002])

        path[-1][-1] = 0.0

        self.rtde_c.moveL(path)

    def move_8flower_triangles(self, center, side=0.05, a=0.5, v=0.05):

        pose = self.get_tcp_pose()
        z = pose[2]
        rx, ry, rz = pose[3:]

        cx, cy = center

        path = []
        half_base = (side*0.4)/2
        h = math.sqrt(pow(side, 2) - pow(half_base, 2))
        degree_angle = math.pi/4
        angles = [0, degree_angle, degree_angle*2, degree_angle*3, degree_angle*4, degree_angle*5, degree_angle*6, degree_angle*7]

        for angle in angles:
            dx = math.cos(angle)
            dy = math.sin(angle)

            px = -dy
            py = dx

            p1 = [cx, cy]  # центр
            p2 = [cx+dx * h+px * side / 2, cy+dy * h+py * side / 2]
            p3 = [cx+dx * h-px * side / 2, cy+dy * h-py * side / 2]

            path.append([p1[0], p1[1], z, rx, ry, rz, v, a, 0.002])
            path.append([p2[0], p2[1], z, rx, ry, rz, v, a, 0.002])
            path.append([p3[0], p3[1], z, rx, ry, rz, v, a, 0.002])
            path.append([p1[0], p1[1], z, rx, ry, rz, v, a, 0.002])

        path[-1][-1] = 0.0

        self.rtde_c.moveL(path)

    def move_triangle(self, side_length=0.005, a=0.3, v=0.02):
        """
        Движение по равностороннему треугольнику в плоскости XY

        side_length — длина стороны (в метрах, 0.05 = 5 см)
        """
        time.sleep(4)

        start_pose = self.get_tcp_pose()  # [x, y, z, rx, ry, rz]

        x0, y0, z0, rx, ry, rz = start_pose

        p1 = [x0, y0, z0, rx, ry, rz]
        p2 = [x0+side_length, y0, z0, rx, ry, rz]

        height = side_length * math.sqrt(3) / 2
        p3 = [x0+side_length / 2, y0+height, z0, rx, ry, rz]

        self.move("movel", p2, a, v)
        self.move("movel", p3, a, v)
        self.move("movel", p1, a, v)

    def speedl(self, tcp_speed, a=0.1, t=10.0):
        try:
            return self.rtde_c.speedL(tcp_speed, a, t)
        except AttributeError:
            raise

    def stop(self, a=0.3):
        try:
            self.rtde_c.speedStop(a)
        except Exception:
            pass

        time.sleep(0.05)

    def move_towards(self, vector, distance, a=0.1, v=0.2):
        current_pose = self.get_tcp_pose()
        if current_pose is None:
            raise RuntimeError("Не удалось получить текущую позицию TCP")

        norm = math.sqrt(vector[0] ** 2+vector[1] ** 2+vector[2] ** 2)
        if norm == 0:
            raise ValueError("Zero direction vector")

        scale = distance / norm

        new_pose = [
            current_pose[0]+vector[0] * scale,  # x
            current_pose[1]+vector[1] * scale,  # y
            current_pose[2]+vector[2] * scale,  # z
            current_pose[3],  # rx
            current_pose[4],  # ry
            current_pose[5]  # rz
        ]

        self.move("movel", new_pose, a, v)

    def move_until_contact(self, direction, force_threshold=30.0, max_time=10.0, a=0.05, v=0.05):
        self.rtde_c.zeroFtSensor()
        time.sleep(0.1)

        start_time = time.time()
        norm = math.sqrt(sum(c*c for c in direction))
        if norm == 0:
            raise ValueError("Zero direction")

        tcp_v = [v * (c / norm) for c in direction] + [0.0, 0.0, 0.0]

        self.speedl(tcp_v, a=a, t=max_time + 1.0)

        try:
            while True:
                forces = self.get_tcp_force()
                if forces is None:
                    time.sleep(0.01)
                    continue
                Fx, Fy, Fz = forces[0], forces[1], forces[2]
                force = math.sqrt(Fx*Fx + Fy*Fy + Fz*Fz)
                if force > force_threshold:
                    print("contact")
                    self.rtde_c.speedStop(0.2)
                    time.sleep(0.1)
                    return True
                if time.time()-start_time > max_time:
                    print("timeout")
                    self.stop(a=0.2)
                    time.sleep(0.1)
                    return False
                time.sleep(0.01)
        finally:
            try:
                self.stop(a=0.2)
            except Exception:
                pass

    def move_circle_xy(self, center, radius, steps=200, a=0.5, v=0.05):
        pose = self.get_tcp_pose()

        z = pose[2]
        rx, ry, rz = pose[3:]

        cx, cy = center

        path = []

        for i in range(steps):
            theta = 4 * math.pi * i / steps

            x = cx+radius * math.cos(theta)
            y = cy+radius * math.sin(theta)

            path.append([x, y, z, rx, ry, rz, v, a, 0.0])

        # ❗ явно замыкаем
        x0 = cx+radius
        y0 = cy
        path.append([x0, y0, z, rx, ry, rz, v, a, 0.0])

        self.rtde_c.moveL(path)

        # ❗ дожим
        self.rtde_c.moveL([x0, y0, z, rx, ry, rz], v, a)

    def parse_bits(self, count: int = 18) -> dict:
        return {f"DI{i}": (self.rtde_r.getActualDigitalInputBits() >> i) & 1 for i in range(count)}

    def close(self):
        try:
            self.rtde_c.stopScript()
        except Exception:
            pass
        try:
            self.rtde_c.disconnect()
        except Exception:
            pass
        try:
            self.rtde_r.disconnect()
        except Exception:
            pass
        try:
            self.rtde_o.disconnect()
        except Exception:
            pass