import sys, time
sys.path.append("C:/Users/Gido/Documents/workspace/development/mobile_additive_manufacturing/src/mobile_additive_manufacturing/control")
from clay_extruder_control import ExtruderClient

# extruderclient connect
ec = ExtruderClient()
ec.connect()
time.sleep(0.5)
print("Connection: ", ec.connected)

# set the motordata
ec.send_motorstate(0, False)

time.sleep(0.5)
ec.close()
print("Connection: ", ec.connected)