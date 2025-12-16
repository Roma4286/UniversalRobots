import socket
import time

from config import DASHBOARD_PORT


class Tool:
    def __init__(self, host, port=DASHBOARD_PORT, delay=0.5):
        self.host = host
        self.port = port
        self.delay = delay
        self.sock = socket.socket()
        self.connect()

    def connect(self):
        self.sock.connect((self.host, self.port))
        print(self.sock.recv(1024).decode().strip())

    def _dashboard_send_cmd(self, cmd):
        self.sock.sendall((cmd + "\n").encode())
        resp = self.sock.recv(1024).decode().strip()
        print(f"> {cmd} -> {resp}")
        time.sleep(self.delay)
        return resp

    def grip(self, name_file: str):
        self._dashboard_send_cmd("stop")
        self._dashboard_send_cmd(f"load {name_file}")
        self._dashboard_send_cmd("play")

    def release(self, name_file: str):
        self._dashboard_send_cmd("stop")
        self._dashboard_send_cmd(f"load {name_file}")
        self._dashboard_send_cmd("play")

    def close(self):
        self.sock.close()