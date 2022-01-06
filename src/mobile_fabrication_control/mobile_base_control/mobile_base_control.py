import time
import math
from compas.robots import Configuration
from compas.robots.model.joint import Joint
from compas_fab.backends import RosClient
from compas_fab.backends.ros import messages
from compas_fab.backends.ros.messages import JointTrajectory, JointTrajectoryPoint, Header
from compas_fab.robots.time_ import Duration
from roslibpy import Message
from roslibpy import Topic
from roslibpy import Service
from roslibpy.core import ServiceRequest


class AttrDict(dict):
    def __init__(self, *args, **kwargs):
        super(AttrDict, self).__init__(*args, **kwargs)
        self.__dict__ = self


class MobileBaseControl():
    def __init__(self, host='localhost', port=9090):
        # self.ros = roslibpy.Ros(host='localhost', port=9090)
        self.ros = RosClient(host=host, port=port)
        self.cmd_vel = AttrDict(linear=AttrDict(x=0.0, y=0.0, z=0.0),
                                angular=AttrDict(x=0.0, y=0.0, z=0.0))
        self.topics = {}

    def topic_subscriber(self, name, msg=None, callback=None):
        if name not in self.topics.keys():
            self.set_topic(name, msg)
        if not self.topics[name].is_subscribed:
            self.topics[name].subscribe(callback)
        return self.topics[name]

    def topic_publisher(self, name, msg=None):
        if name not in self.topics.keys():
            self.set_topic(name, msg)
        if not self.topics[name].is_advertised:
            self.topics[name].advertise()
        return self.topics[name]

    def get_topic(self, name):
        return self.topics[name]

    def set_topic(self, name, msg):
        self.topics[name] = Topic(self.ros, name, msg)
        return self.topics[name]

    def connect(self):
        self.ros.run()
        print("Is ROS connected? ", self.ros.is_connected)

    def disconnect(self):
        self.ros.close()

    def load_from_robot(self):
        self.robot = self.ros.load_robot()

    def load_from_urdf(self):
        raise NotImplementedError

    def condition_odometry(self):
        # callback = some definition
        # self.topic_subscriber('/robot/robotnik_base_control', callback)
        # check the odom value vs beginning
        raise NotImplementedError

    def condition_laser(self):
        # self.topic_subscriber('/robot/front_3d_laser/points', callback)
        raise NotImplementedError

    def print_msg_callback(self, message):
        print(message['data'])

    def echo_joint_states(self):
        # jst = self.topic_subscriber(name="/robot/arm/scaled_pos_traj_controller/state",
        #                       msg="control_msgs/JointTrajectoryControllerState",
        #                       callback=lambda message: print(message['data']))
        # time.sleep(2)
        # jst.unsubscribe()
        pass

    def list_controllers(self):
        list_controllers_service = Service(self.ros, "/robot/controller_manager/list_controllers", "/robot/controller_manager/list_controllers")
        request = ServiceRequest()
        print(list_controllers_service.call(request))

    def move_forward(self, vel=1, dist=5.0):
        move_base = self.topic_publisher("/robot/cmd_vel", "geometry_msgs/Twist")
        self.cmd_vel.linear.x = vel
        t0 = time.time()
        while dist > (time.time()-t0)*abs(vel):
            move_base.publish(Message(self.cmd_vel))
            time.sleep(0.1)
        move_base.unadvertise()

    def move_backward(self, vel=-1, dist=5):
        self.move_forward(vel=vel, dist=dist)

    def move_radial(self, deg=90, vel=1, dist=5):
        move_base = self.topic_publisher("/robot/cmd_vel", "geometry_msgs/Twist")
        x_vel = math.cos(math.radians(deg))*vel
        y_vel = math.sin(math.radians(deg))*vel
        self.cmd_vel.linear.x = x_vel
        self.cmd_vel.linear.y = y_vel
        t0 = time.time()
        while dist > (time.time()-t0)*abs(vel):
            move_base.publish(Message(self.cmd_vel))
            time.sleep(0.1)
        move_base.unadvertise()

    def arm_move_joint(self, configuration, max_velocity=[0.2,0.2,0.2,0.2,0.2,0.2], acceleration=[1,1,1,1,1,1]):
        joint_state_publisher = Topic(self.ros, "/robot/arm/scaled_pos_traj_controller/command", "trajectory_msgs/JointTrajectory")
        joint_state_publisher.advertise()
        treq = [pos/vel for pos, vel in zip(configuration.joint_values, max_velocity)]
        treq_max = max(*treq)
        vreq = [pos/treq_max for pos in configuration.joint_values]
        rostime = self.ros.get_time()
        rostime1 = Duration.from_data(rostime)
        rostime1.secs += treq_max
        rostime1.nsecs += treq_max
        rostime2 = Duration.from_data(rostime1.data)
        rostime2.secs += treq_max
        rostime2.nsecs += treq_max

        jtp = JointTrajectoryPoint(positions=configuration.joint_values, velocities=[1,1,1,1,1,1], accelerations=[1,1,1,1,1,1], time_from_start=rostime1.data)
        jtp0 = JointTrajectoryPoint(positions=[0,0,0,0,0,0], velocities=[0,0,0,0,0,0], accelerations=[0,0,0,0,0,0], time_from_start=rostime)
        jtp1 = JointTrajectoryPoint(positions=[0,0,0,0,0,0], velocities=[0,0,0,0,0,0], accelerations=[0,0,0,0,0,0], time_from_start=rostime2.data)
        jt = JointTrajectory(header=Header(stamp=rostime, frame_id=''), joint_names=['robot_arm_shoulder_pan_joint', 'robot_arm_shoulder_lift_joint', 
                                                           'robot_arm_elbow_joint', 'robot_arm_wrist_1_joint', 
                                                           'robot_arm_wrist_2_joint', 'robot_arm_wrist_3_joint'], points=[jtp0,jtp,jtp1])
        # print(jtp.msg)
        # print(jt.msg)
        # msg={'header': {'seq': 0, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': '/world'},
        #      'joint_names': ['robot_arm_shoulder_pan_joint', 'robot_arm_shoulder_lift_joint',
        #                      'robot_arm_elbow_joint', 'robot_arm_wrist_1_joint',
        #                      'robot_arm_wrist_2_joint', 'robot_arm_wrist_3_joint'],
        #      'points': [{'positions': [0.0, -1.570796, 1.570796, 0.0, -1.570796, 0.0], 'velocities': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 'accelerations': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 'time_from_start': {'secs': 0, 'nsecs': 0}}]}
        print(jt.msg)
        t0 = time.time()
        while time.time()-t0 < treq_max*5:
            joint_state_publisher.publish(jt.msg)
            time.sleep(0.01)
        joint_state_publisher.unadvertise()

    def set_lift_height(self, height):
        lift_topic = self.topic_publisher("/robot/lift_controller/command",
                                          "std_msgs/Float64")
        t0 = time.time()
        while time.time()-t0 < 2:
            lift_topic.publish({"data": height})
        lift_topic.unadvertise()

    def rotate_in_place(self, deg=90, vel=0.1):
        pass

    def stop_robot(self):
        pass


if __name__ == "__main__":
    mb = MobileBaseControl(host='192.168.0.12', port=9090)
    mb.connect()
    time.sleep(1)
    # print(mb.ros.get_topics())
    # mb.list_controllers()
    # print(mb.ros.get_nodes())
    # print(mb.ros.get_services())
    mb.echo_joint_states()
    # config = Configuration()
    # config = Configuration.from_revolute_values([1.57079, 1.57079, 1.57079, 1.57079,1.57079, 1.57079])
    # print(config.joint_values)
    # mb.arm_move_joint(config)
    # config = Configuration.from_revolute_values([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # mb.arm_move_joint(config)
    # mb.set_lift_height(0.2)
    # mb.move_forward()
    # time.sleep(1)
    # mb.move_radial(deg=90)
    time.sleep(1)
    mb.disconnect()

