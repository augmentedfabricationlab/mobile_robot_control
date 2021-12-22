import sys, time
sys.path.append("C:/Users/Gido/Documents/workspace/development/mobile_additive_manufacturing/src/mobile_additive_manufacturing/control")
from clay_extruder_control import ExtruderClient

# globals
microstep = 16
enable_pin = 30
max_rot_sec = 16
rot_sec = 1
max_speed = 200*microstep*max_rot_sec
run_speed = 200*microstep*rot_sec
#run_speed = 6400
run_speed = -800
# extruderclient connect
ec = ExtruderClient()
ec.connect()
time.sleep(0.5)
print("Connection: ", ec.connected)

ec.send_motordata(1, max_speed, run_speed, False)
time.sleep(0.5)
ec.close()
print("Connection: ", ec.connected)

time.sleep(5)
ec.connect()
time.sleep(0.5)
print("Connection: ", ec.connected)

# set the motordata
ec.send_motorstate(0, 0, False)

time.sleep(0.5)
ec.close()