import socket
import struct
import time
from .message_types import msg_types_dict
from .message_types import MSG_EXECUTED, MSG_STOP, MSG_INFO, MSG_DODATA
from .message_types import MSG_MOTORDATA, MSG_MOTORSTATE

__all__ = [
    "ExtruderClient"
]


class ExtruderClient():
    def __init__(self, host="192.168.10.50", port=50001):
        self.host = host
        self.port = port
        self.connected = False

        self.msg_rcv = bytearray([])
        self.info_msg = ""
        self.header_byteorder = ">lll"

    def clear(self):
        self.msg_rcv = bytearray([])

    def connect(self):
        if not hasattr(self, "sock"):
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            connected_msg = "Connected to server {} on port {}"
            print(connected_msg.format(self.host, self.port))
            self.connected = True

    def close(self):
        if hasattr(self, "sock"):
            self.sock.close()
            self.connected = False
            del self.sock

    # Send commands

    def send_stop(self, wait_resp=False):
        return self.__send(MSG_STOP, wait_for_response=wait_resp)

    def send_set_do(self, pin=0, state=0, wait_resp=False):
        msg = [pin, state]
        return self.__send(MSG_DODATA, 8, msg, wait_resp)

    def send_motordata(self, state, max_speed, speed, wait_resp=False):
        msg = [state, max_speed, speed]
        return self.__send(MSG_MOTORDATA, 12, msg, wait_resp)

    def send_motorstate(self, state, wait_resp=False):
        msg = [state]
        return self.__send(MSG_MOTORSTATE, 4, msg, wait_resp)

    def send_get_arduino_info(self, wait_resp=False):
        return self.__send(MSG_INFO, wait_for_response=wait_resp)

    # Get commands

    def get_msg_stop(self, wait_resp=False):
        return self.__get_msg(MSG_STOP, wait_for_response=wait_resp)

    def get_msg_set_do(self, pin=0, state=0, wait_resp=False):
        msg = [pin, state]
        return self.__get_msg(MSG_DODATA, 8, msg, wait_resp)

    def get_msg_motordata(self, state, max_speed, speed, wait_resp=False):
        msg = [state, max_speed, speed]
        return self.__get_msg(MSG_MOTORDATA, 12, msg, wait_resp)

    def get_msg_motorstate(self, state, wait_resp=False):
        msg = [state]
        return self.__get_msg(MSG_MOTORSTATE, 4, msg, wait_resp)

    def get_msg_arduino_info(self, wait_resp=False):
        return self.__get_msg(MSG_INFO, wait_for_response=wait_resp)

    # Internal commands

    def __get_msg(self, msg_type, msg_len=0, msg=None,
                  wait_for_response=True, packed=False):
        msg_list = [msg_type, msg_len, int(wait_for_response)]
        if msg is not None:
            msg_list.extend(msg)
        if packed:
            msg_byteorder = self.header_byteorder + msg_types_dict[msg_type][1]
            packed_msg = struct.pack(msg_byteorder, *msg_list)
            return packed_msg
        else:
            return msg_list

    def __send(self, msg_type, msg_len=0, msg=None, wait_for_response=True):
        __msg = self.__get_msg(msg_type, msg_len, msg, wait_for_response, True)
        self.sock.send(__msg)
        headers, msgs = [], []
        while wait_for_response:
            header, msg = self.__read()
            if header is not None:
                rcv_msg_type = msg_types_dict[header[0]][0]
                print("{} received from Arduino.".format(rcv_msg_type))
                print("Message: {}".format(msg))
                headers.append(header)
                msgs.append(msg)
                wait_for_response = header[0] != MSG_EXECUTED
                print("Still waiting? -> ", wait_for_response)
            time.sleep(0.1)
        else:
            return(headers, msgs)

    def __read(self):
        try:
            self.msg_rcv.extend(self.sock.recv(1))
            print(self.msg_rcv)
            header = struct.unpack_from(self.header_byteorder, self.msg_rcv)
            print(header)
        except struct.error:
            return None, None
        else:
            if header[1] > 0:
                rcv_msg_byteorder = msg_types_dict[header[0]][2]
                raw_msg = self.sock.recv(header[1])
                msg = struct.unpack_from(rcv_msg_byteorder, raw_msg)
            else:
                msg = ""
            print(msg)
            self.clear()
            return header, msg


if __name__ == "__main__":
    ec = ExtruderClient(host="192.168.0.220", port=50004)
    ec.connect()
    time.sleep(0.5)
    print("Connection: ", ec.connected)
    wait = False
    # ec.send_motordata(0, 0, 1600, 800, wait)
    # ec.send_motorstate(0, 1, wait)
    # time.sleep(5)
    # ec.send_motorstate(0, 0, wait)
    ec.send_set_do(8,1)
    time.sleep(0.5)
    ec.close()
    print("Connection: ", ec.connected)
