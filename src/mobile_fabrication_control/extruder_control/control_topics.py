import time
import roslibpy

def publisher(ros, topic, arg_type, arg):
    publisher = roslibpy.Topic(ros, topic, arg_type)
    publisher.publish(roslibpy.Message({'data': arg}))
    time.sleep(1)
    publisher.unadvertise()

topic_lib = {
    '/fan_toggle' : 'std_msgs/Bool',
    '/air_toggle' : 'std_msgs/Bool',
    '/extruder_toggle' : 'std_msgs/Bool',
    '/stepper_speed' : 'std_msgs/Float32',
    '/stepper_accel' : 'std_msgs/Float32',
}

def toggle_fan(ros, arg=False):
    publisher(ros, '/fan_toggle', topic_lib['/fan_toggle'], arg)

def toggle_air(ros, arg=False):
    publisher(ros, '/air_toggle', topic_lib['/air_toggle'], arg)

def toggle_extruder(ros, arg=False, speed=None, acc=None):
    if speed:
        set_extruder_speed(ros, speed)
    if acc:
        set_extruder_accel(ros, acc)
    publisher(ros, '/extruder_toggle', topic_lib['/extruder_toggle'], arg)

def set_extruder_speed(ros, speed):
    publisher(ros, '/stepper_speed', topic_lib['/stepper_speed'], speed)

def set_extruder_accel(ros, acc):
    publisher(ros, '/stepper_accel', topic_lib['/stepper_accel'], acc)

if __name__ == '__main__':
    ros = roslibpy.Ros(host='192.168.10.12', port=9090)
    ros.run()
    print(ros.is_connected)
    ros.terminate()