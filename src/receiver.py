import socket
import threading

import msgpack

import messageHandler
from gvh import Gvh
from message import to_msg


class Receiver(threading.Thread):
    """
    __stop_event: threading.Event
    __ip: str
    __port: int
    """

    """
    TODO: Merge with Sender potentially
    """

    def __init__(self, ip: str, port: int):
        super(Receiver, self).__init__()
        self.__agent_gvh = None
        self.__ip = ip
        self.__port = port
        self.__stop_event = threading.Event()

    def stop(self):  # -> NoReturn:
        """
         a flag to set to to safely exit the thread
        :return:
        """
        print("stopping receiver")
        self.__stop_event.set()

    def stopped(self):  # -> NoReturn:
        """
        set the stop flag
        :return:
        """
        return self.__stop_event.is_set()

    @property
    def agent_gvh(self):
        """

        :return:
        """
        return self.__agent_gvh

    @agent_gvh.setter
    def agent_gvh(self, agent_gvh: Gvh):
        """

        :param agent_gvh:
        :return:
        """
        self.__agent_gvh = agent_gvh

    @property
    def ip(self) -> str:
        """
        getter method for ip
        :return: string ip
        """
        return self.__ip

    @ip.setter
    def ip(self, ip: str):  # -> NoReturn:
        """
        setter method for ip
        """
        self.__ip = ip

    @property
    def port(self) -> int:
        """
        getter method for port
        :return: int port
        """
        return self.__port

    @port.setter
    def port(self, port: int):  # -> NoReturn:
        """
        setter method for ip
        """
        self.__port = port

    def recv(self):
        """

        :return:
        """
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_sock.bind((self.ip, self.port))
        data, addr = server_sock.recvfrom(1024)
        server_sock.close()
        return to_msg(msgpack.unpackb(data).decode()).content

    def run(self):
        """

        :return:
        """
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        server_sock.bind((self.ip, self.port))
        server_sock.settimeout(2)

        while not self.stopped():
            try:
                data, addr = server_sock.recvfrom(1024)
                msg = to_msg(msgpack.unpackb(data).decode())
                messageHandler.message_handler[msg.m_type](msg, self.agent_gvh)
            except socket.timeout:
                pass
        server_sock.close()