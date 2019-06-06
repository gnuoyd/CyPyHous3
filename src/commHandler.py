from message import *
from receiver import *
from sender import *


class CommHandler(object):
    """
    Communication handling thread. Has a receiver , which will receive all messages.
    We also have a sender class, with a send method.
    TODO: figure out design
    """

    def __init__(self, ip: str, recv_port: int, snd_port: int, pid: int):
        self.__receiver = Receiver(ip, recv_port)
        self.__sender = Sender(ip, snd_port)
        self.__pid = pid
        self.__receiver.start()

    @property
    def pid(self) -> int:
        """
        getter method for pid
        :return:
        """
        return self.__pid

    @property
    def sender(self) -> Sender:
        """
        getter method for sender
        :return: Sender
        """
        return self.__sender

    @sender.setter
    def sender(self, sender: Sender) -> None:
        """
        setter method for sender
        :param sender:
        :return:
        """
        self.__sender = sender

    @property
    def receiver(self) -> Receiver:
        """
        getter method for receiver
        :return: Receiver
        """
        return self.__receiver

    @receiver.setter
    def receiver(self, receiver: Receiver) -> None:
        """
        setter method for receiver
        :param receiver:
        :return:
        """
        self.__receiver = receiver

    def send(self, message: Message) -> None:
        """
        sending message using the sender object
        :param message: message
        :return:
        """
        self.sender.send(message)