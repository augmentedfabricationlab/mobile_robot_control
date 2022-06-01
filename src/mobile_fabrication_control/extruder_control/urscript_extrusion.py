import time
from ur_fabrication_control.direct_control import URScript
from .extruder_mixins import ExtruderMixins

__all__ = [
    "URScript_Extrusion",
    "stop_extruder",
    "set_extruder_digital_out",
    "toggle_fan",
    "toggle_air",
    "toggle_drv8825",
    "toggle_extruder",
    "set_extruder_data",
    "get_extruder_info"
]


class URScript_Extrusion(URScript, ExtruderMixins):
    def __init__(self, ur_ip=None, ur_port=None, ext_ip=None, ext_port=None):
        super(URScript_Extrusion, self).__init__(ur_ip, ur_port)
        self.ext_setup(ext_ip, ext_port)


def __wrap(func, *args, **kwargs):
    urscript = URScript_Extrusion(*args)
    urscript.start()
    urscript.open_socket_extruder()
    func(urscript, **kwargs)
    urscript.close_socket_extruder()
    urscript.end()
    urscript.generate()
    urscript.send_script()


def stop_extruder(ur_ip, ur_port, ext_ip, ext_port):
    ''' Sends a stop command to the extruder '''
    __wrap(URScript_Extrusion.stop_extruder,
           ur_ip, ur_port, ext_ip, ext_port)


def set_extruder_digital_out(ur_ip, ur_port, ext_ip, ext_port,
                             pin=0, state=0, wait_for_response=False):
    # Sends a command to enable or disable a pin
    __wrap(URScript_Extrusion.set_extruder_digital_out,
           ur_ip, ur_port, ext_ip, ext_port,
           pin=pin, state=state, wait_for_response=wait_for_response)


def toggle_fan(ur_ip, ur_port, ext_ip, ext_port,
               pin=34, state=0, wait_for_response=False):
    # Sends a command to enable or disable the fan
    __wrap(URScript_Extrusion.toggle_fan,
           ur_ip, ur_port, ext_ip, ext_port,
           pin=pin, state=state, wait_for_response=wait_for_response)


def toggle_air(ur_ip, ur_port, ext_ip, ext_port,
               pin=32, state=0, wait_for_response=False):
    # Sends a command to enable or disable the air
    __wrap(URScript_Extrusion.toggle_air,
           ur_ip, ur_port, ext_ip, ext_port,
           pin=pin, state=state, wait_for_response=wait_for_response)


def toggle_drv8825(ur_ip, ur_port, ext_ip, ext_port,
                   pin=30, state=0, wait_for_response=False):
    __wrap(URScript_Extrusion.toggle_drv8825,
           ur_ip, ur_port, ext_ip, ext_port,
           pin=pin, state=state, wait_for_response=wait_for_response)


def toggle_extruder(ur_ip, ur_port, ext_ip, ext_port,
                    state=0, wait_for_response=False):
    __wrap(URScript_Extrusion.toggle_extruder,
           ur_ip, ur_port, ext_ip, ext_port,
           state=state, wait_for_response=wait_for_response)


def set_extruder_data(ur_ip, ur_port, ext_ip, ext_port,
                      state=0, max_speed=4000, speed=400):
    __wrap(URScript_Extrusion.set_extruder_data,
           ur_ip, ur_port, ext_ip, ext_port,
           state=state, max_speed=max_speed, speed=speed)


def get_extruder_info(ur_ip, ur_port, ext_ip, ext_port,
                      wait_for_response=False):
    __wrap(URScript_Extrusion.get_extruder_info,
           ur_ip, ur_port, ext_ip, ext_port,
           wait_for_response=wait_for_response)


if __name__ == "__main__":
    addresses = ["192.168.10.20", 30002, "192.168.10.50", 50001]
    toggle_fan(*addresses, state=1)
    time.sleep(2)
    toggle_air(*addresses, state=1)
    time.sleep(2)
    toggle_drv8825(*addresses, state=1)
    toggle_extruder(*addresses, state=1)
    time.sleep(10)
    toggle_extruder(*addresses, state=0)
    toggle_drv8825(*addresses, state=0)
    time.sleep(2)
    toggle_air(*addresses, state=0)
    time.sleep(2)
    toggle_fan(*addresses, state=0)
