import struct
from .extruderclient import ExtruderClient

__all__ = [
    "ExtruderMixins"
]


class ExtruderMixins(object):
    def ext_setup(self, ext_ip="192.168.10.50", ext_port=50001,
                  ext_name="extruder_0"):
        self.ext_client = ExtruderClient()
        self.ext_ip = ext_ip
        self.ext_port = ext_port
        self.ext_name = ext_name

    @property
    def ext_addr(self):
        return (self.ext_ip, self.ext_port)

    def open_socket_extruder(self, host="192.168.10.50", port=50001,
                             name="extruder_0"):
        ''' Open connection with the extruder
        (Only a single extruder has been integrated)'''
        if ((self.ext_ip != host or self.ext_port != port) and
                (host != "192.168.10.50" and port != 50001)):
            # Port is different than defined, values will be overwritten
            self.ext_ip, self.ext_port = host, port
            addr_msg = "IP and/or port changed, was {} at {}"
            print(addr_msg.format(self.ext_ip, self.ext_port))
        if self.ext_name != name and name != "extruder_0":
            # Name socket different than defined, name will be overwritten
            sock_msg = "Socket name changed from {} to {}"
            print(sock_msg.format(self.ext_name, name))
            self.ext_name = name
        self.socket_open(self.ext_ip, self.ext_port, self.ext_name)

    def close_socket_extruder(self, socket_name="extruder_0"):
        self.socket_close(socket_name)

    def wait_for_response(self, timeout=10):
        self.add_lines([
            '\twait_for_response = True',
            '\tattempts = 0',
            '\twhile wait_for_response == True:',
            '\t\theader = socket_read_binary_integer' +
            '(3, socket_name="{}", timeout={})'.format(self.ext_name, timeout),
            '\t\ttextmsg(header)',
            '\t\tif header[1]>0:',
            '\t\t\tif header[0]==3:',
            '\t\t\t\tarduino_ip = socket_read_string' +
            '(socket_name="{}", timeout={})'.format(self.ext_name, timeout),
            '\t\t\t\tarduino_gateway = socket_read_string' +
            '(socket_name="{}", timeout={})'.format(self.ext_name, timeout),
            '\t\t\t\tarduino_subnet = socket_read_string' +
            '(socket_name="{}", timeout={})'.format(self.ext_name, timeout),
            '\t\t\t\tarduino_port = socket_read_binary_integer' +
            '(1, socket_name="{}", timeout={})'.format(self.ext_name, timeout),
            '\t\t\t\ttextmsg(arduino_ip)',
            '\t\t\t\ttextmsg(arduino_gateway)',
            '\t\t\t\ttextmsg(arduino_subnet)',
            '\t\t\t\ttextmsg(arduino_port)',
            '\t\t\tend',
            '\t\telse:',
            '\t\t\tattempts = attempts + 1',
            '\t\tend',
            '\t\twait_for_response = (header[0]!=1)',
            '\t\tif attempts>5:',
            '\t\t\twait_for_response = False',
            '\t\tend',
            '\tend'
        ])

    def stop_extruder(self, wait_for_response=False):
        msg = self.ext_client.get_msg_stop(wait_for_response)
        self.socket_send_ints(msg, socket_name=self.ext_name,
                              address=self.ext_addr)
        if wait_for_response:
            self.wait_for_response()

    def set_extruder_digital_out(self, pin=0, state=0,
                                 wait_for_response=False):
        msg = self.ext_client.get_msg_set_do(pin, state, wait_for_response)
        self.socket_send_ints(msg, socket_name=self.ext_name,
                              address=self.ext_addr)
        if wait_for_response:
            self.wait_for_response()

    def toggle_fan(self, pin=34, state=0, wait_for_response=False):
        self.set_extruder_digital_out(pin, state, wait_for_response)

    def toggle_air(self, pin=32, state=0, wait_for_response=False):
        self.set_extruder_digital_out(pin, state, wait_for_response)

    def toggle_drv8825(self, pin=30, state=0, wait_for_response=False):
        self.set_extruder_digital_out(pin, state, wait_for_response)

    def toggle_extruder(self, state=0, wait_for_response=False):
        msg = self.ext_client.get_msg_motorstate(state, wait_for_response)
        self.socket_send_ints(msg, socket_name=self.ext_name,
                              address=self.ext_addr)
        if wait_for_response:
            self.wait_for_response()

    def set_extruder_data(self, state=0, max_speed=4000, speed=400,
                          wait_for_response=False):
        msg = self.ext_client.get_msg_motordata(state, max_speed, speed,
                                                wait_for_response)
        self.socket_send_ints(msg[:4], socket_name=self.ext_name,
                              address=self.ext_addr)
        floats = [max_speed, speed]
        float_bytes = list(bytearray(struct.pack(">ff", *floats)))
        self.socket_send_bytes(float_bytes, self.ext_name, self.ext_addr)

        if wait_for_response:
            self.wait_for_response()

    def get_extruder_info(self, wait_for_response=True):
        msg = self.ext_client.get_msg_arduino_info(wait_for_response)
        self.socket_send_ints(msg, socket_name=self.ext_name,
                              address=self.ext_addr)
        if wait_for_response:
            self.wait_for_response()
