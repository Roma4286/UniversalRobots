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
            return self.rtde_c.moveJ(target, speed=v, acceleration=a)
        elif move_type.lower() == "movel" or move_type.lower() == "movel":
            return self.rtde_c.moveL(target, speed=v, acceleration=a)
        else:
            raise ValueError("Unsupported move type: " + move_type)

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

    def move_until_contact(self, direction, force_threshold=30.0, max_time=10.0, a=0.05, v=0.05):
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
