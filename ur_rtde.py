# robot_rtde.py
import time
import math

try:
    # наиболее распространённые импорты для RTDE-based control/receive
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface
except Exception as e:
    raise ImportError("Не удалось импортировать RTDE-клиент. Установи rtde/ur-rtde. Ошибка: " + str(e))


class RobotRTDE:
    def __init__(self, host, port_control=30002, port_receive=30004, timeout=5.0):
        """
        host: IP робота
        port_control: порт управления (обычно 30002 для RTDE control wrapper)
        port_receive: порт RTDE receive (обычно 30004)
        """
        self.host = host
        self.timeout = timeout

        # Подключаем receive (чтение данных) и control (движение)
        self.rtde_r = RTDEReceiveInterface(self.host, port_receive)
        self.rtde_c = RTDEControlInterface(self.host, port_control)

        # небольшая пауза для установления соединения
        time.sleep(0.1)

        if not self.is_connected():
            raise RuntimeError("Не удалось подключиться к RTDE интерфейсу на " + self.host)

    def is_connected(self):
        try:
            # простая проверка чтения
            _ = self.rtde_r.getActualTCPPose()
            return True
        except Exception:
            return False

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

    def move(self, move_type: str, target, a=1.0, v=0.1):
        """
        move_type: 'moveJ' или 'moveL'
        target: список joint-углов [6] или pose [6]
        """
        if move_type.lower() == "movej":
            return self.rtde_c.moveJ(target, speed=v, acceleration=a)
        elif move_type.lower() == "movel" or move_type.lower() == "movel":
            return self.rtde_c.moveL(target, speed=v, acceleration=a)
        else:
            raise ValueError("Unsupported move type: " + move_type)

    def speedl(self, tcp_speed, a=0.1, t=10.0):
        """Запускает линейную скорость в TCP"""
        # метод в rtde_control может называться speedL / speedl в зав. от версии
        try:
            return self.rtde_c.speedL(tcp_speed, a, t)
        except AttributeError:
            # fallback: послать через rtde_c.sendScript или использовать move commands
            raise

    def stop(self, a=0.2):
        try:
            self.rtde_c.stopL(a)
        except Exception:
            # fallback: ничего не делаем — caller должен обработать
            pass

    def move_until_contact(self, direction, force_threshold=30.0, max_time=5.0, a=0.05, v=0.05):
        """
        Движение по вектору direction [dx,dy,dz] до превышения силы.
        Контроль по RTDE force. max_time защищает от "ухода".
        """
        start_time = time.time()
        # нормируем вектор и масштабируем до желаемой TCP скорости v
        norm = math.sqrt(sum(c*c for c in direction))
        if norm == 0:
            raise ValueError("Zero direction")

        tcp_v = [v * (c / norm) for c in direction] + [0.0, 0.0, 0.0]

        # запускаем speedl
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
                    # мягкая остановка
                    self.stop(a=0.2)
                    return True
                if time.time() - start_time > max_time:
                    # timeout — безопасная остановка и выход
                    self.stop(a=0.2)
                    return False
                time.sleep(0.01)
        finally:
            # убедиться, что скорость остановлена
            try:
                self.stop(a=0.2)
            except Exception:
                pass

    def close(self):
        try:
            self.rtde_c.stopScript()  # если есть
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


# robotIP = "172.17.0.2:6800"
PRIMARY_PORT = 30001
SECONDARY_PORT = 30002
REALTIME_PORT = 30003

robotIP = "ursim"
robot = RobotRTDE(robotIP)
print(robot.get_tcp_pose())

robot.move("movej", [.148176530408, -.770704002574, -.312483911963, .131255194183, -3.134044456313, .004624001678])
time.sleep(10)
print(robot.get_tcp_pose())
