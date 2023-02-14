import time
import math
from compas.geometry import Frame, Point, Quaternion, Vector, Transformation
from compas_fab.backends import RosClient
from compas_fab.backends.ros.messages import JointTrajectory, JointTrajectoryPoint, Header
from compas_fab.robots.time_ import Duration
from roslibpy import Message, Topic, Service, tf
from roslibpy.core import ServiceRequest

__all__ = [
    "AttrDict",
    "MobileRobotClient"
]

class AttrDict(dict):
    def __init__(self, *args, **kwargs):
        super(AttrDict, self).__init__(*args, **kwargs)
        self.__dict__ = self

class MobileRobotClient(object):
    def __init__(self, host='localhost', port=9090):
        """_summary_

        Args:
            host (str, optional): IP address of ROS master. Defaults to 'localhost'.
            port (int, optional): Port of ROS master. Defaults to 9090.
        """
        self.ros_client = RosClient(host=host, port=port)
        
        self.topics = {}
        self.services = {}
        self.tf_clients = {}
        self.action_clients = {}
        
        self.cmd_vel = AttrDict(linear=AttrDict(x=0.0, y=0.0, z=0.0),
                                angular=AttrDict(x=0.0, y=0.0, z=0.0))
        self.tf_frame = None
        self.robot_frame = Frame.worldXY()
        
    def connect(self):
        """_summary_
        """
        self.ros_client.run()
        print("Is ROS connected? ", self.ros_client.is_connected)

    def disconnect(self):
        """_summary_
        """
        self.ros_client.close()
        
    def tf_subscribe(self, target_frame, reference_frame):
        """_summary_

        Args:
            target_frame (str): Name of the target frame requested.
            reference_frame (str): Name of the reference frame requested.
        """
        if not self.tf_clients.get(reference_frame):
            tf_client = tf.TFClient(self.ros_client, fixed_frame=reference_frame, angular_threshold=0.0, rate=10.0)
            self.tf_clients[reference_frame] = tf_client
        else:
            tf_client = self.tf_clients.get(reference_frame)
        tf_client.subscribe(target_frame, self._receive_tf_frame_callback)
        
    def _receive_tf_frame_callback(self, message):
        """_summary_

        Args:
            message (_type_): _description_
        """
        pose_point = Point(message['translation']['x'], message['translation']['y'], message['translation']['z'])
        pose_quaternion = Quaternion(message['rotation']['w'], message['rotation']['x'], message['rotation']['y'], message['rotation']['z'])
        pose_frame = Frame.from_quaternion(pose_quaternion, pose_point)
        self.tf_frame = pose_frame
        
    def clean_tf_frame(self):
        self.tf_frame = None

    def tf_unsubscribe(self, target_frame, reference_frame):
        """_summary_

        Args:
            target_frame (_type_): _description_
            reference_frame (_type_): _description_
        """
        if self.tf_clients.get(reference_frame):
            tf_client = self.tf_clients.get(reference_frame)
            try:
                tf_client.unsubscribe(target_frame, 0)
            except TypeError:
                pass
        
    def action_subscribe(self, server_name, action_name, timeout=None):
        pass

    def topic_subscribe(self, topic_name, msg_type=None, callback=None):
        if topic_name not in self.topics.keys():
            self.set_topic(topic_name, msg_type)
        if not self.topics[topic_name].is_subscribed:
            self.topics[topic_name].subscribe(callback)
            
    def topic_unsubscribe(self, topic_name):
        if topic_name in self.topics.keys():
            if self.topics[topic_name].is_subscribed:
                self.topics[topic_name].unsubscribe()
            self.remove_topic(topic_name)

    def topic_publish(self, topic_name, msg_type=None):
        if topic_name not in self.topics.keys():
            self.set_topic(topic_name, msg_type)
        if not self.topics[topic_name].is_advertised:
            self.topics[topic_name].advertise()
            
    def topic_unpublish(self, topic_name):
        if topic_name in self.topics.keys():
            if self.topics[topic_name].is_advertised:
                self.topics[topic_name].unadvertise()
            self.remove_topic(topic_name)

    def get_topic(self, topic_name):
        return self.topics[topic_name]

    def set_topic(self, topic_name, msg_type):
        self.topics[topic_name] = Topic(self.ros_client, topic_name, msg_type)
        return self.topics[topic_name]
    
    def remove_topic(self, topic_name):
        self.topics.pop(topic_name)
    
    def print_msg_callback(self, message):
        print(message['data'])

    def load_from_robot(self):
        self.robot = self.ros_client.load_robot()

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
        
    def cmd_vel_clear(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = 0.0

    def echo_joint_states(self):
        # jst = self.topic_subscriber(name="/robot/arm/scaled_pos_traj_controller/state",
        #                       msg="control_msgs/JointTrajectoryControllerState",
        #                       callback=lambda message: print(message['data']))
        # time.sleep(2)
        # jst.unsubscribe()
        pass

    def echo_robot_odom(self):
        pass
        # rod = self.topic_subscriber(name="/robot/odom",
        #                             msg="geometry_msgs/Pose3D",
        #                             callback=lambda message: print(message['data']))

    def list_controllers(self):
        list_controllers_service = Service(self.ros_client, "/robot/controller_manager/list_controllers", "/robot/controller_manager/list_controllers")
        request = ServiceRequest()
        print(list_controllers_service.call(request))

    def move_forward(self, vel=0.01, dist=0.1):
        move_base = self.topic_publish("/robot/cmd_vel", "geometry_msgs/Twist")
        self.cmd_vel.linear.x = vel*(dist/abs(dist))
        t0 = time.time()
        while abs(dist) > (time.time()-t0)*vel:
            move_base.publish(Message(self.cmd_vel))
            time.sleep(0.01)
        self.cmd_vel_clear()
        T = Transformation.from_frame(Frame([dist,0,0], [1,0,0], [0,1,0]))
        self.robot_frame.transform(T)
        move_base.unadvertise()

    def move_backward(self, vel=0.01, dist=-0.1):
        self.move_forward(vel=vel, dist=-dist)

    def move_radial(self, deg=90, vel=0.1, dist=0.1):
        move_base = self.topic_publish("/robot/cmd_vel", "geometry_msgs/Twist")
        x_vel = math.cos(math.radians(deg))*vel
        y_vel = math.sin(math.radians(deg))*vel
        self.cmd_vel.linear.x = x_vel
        self.cmd_vel.linear.y = y_vel
        t0 = time.time()
        while dist > (time.time()-t0)*abs(vel):
            move_base.publish(Message(self.cmd_vel))
            time.sleep(0.1)
        self.cmd_vel_clear()
        T = Transformation.from_frame(Frame([dist*x_vel/vel,dist*y_vel/vel,0], [1,0,0], [0,1,0]))
        self.robot_frame.transform(T)
        move_base.unadvertise()

    def arm_move_joint(self, configuration, max_velocity=[0.2,0.2,0.2,0.2,0.2,0.2], acceleration=[1,1,1,1,1,1]):
        joint_state_publisher = Topic(self.ros_client, "/robot/arm/scaled_pos_traj_controller/command", "trajectory_msgs/JointTrajectory")
        joint_state_publisher.advertise()
        treq = [pos/vel for pos, vel in zip(configuration.joint_values, max_velocity)]
        treq_max = max(*treq)
        vreq = [pos/treq_max for pos in configuration.joint_values]
        rostime = self.ros_client.get_time()
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
        lift_topic = self.topic_publish("/robot/lift_joint_position_controller/command",
                                          "std_msgs/Float64")
        t0 = time.time()
        while time.time()-t0 < 2:
            lift_topic.publish({"data": height})
        lift_topic.unadvertise()

    def rotate_in_place(self, rad=(math.pi/2), vel=0.01):
        move_base = self.topic_publish("/robot/cmd_vel", "geometry_msgs/Twist")
        self.cmd_vel.angular.z = vel
        t0 = time.time()
        while abs(rad) > (time.time()-t0)*abs(vel):
            move_base.publish(Message(self.cmd_vel))
            time.sleep(0.01)
        self.cmd_vel_clear()
        x_vec = [math.cos(rad), math.sin(rad), 0]
        y_vec = [-math.sin(rad), math.cos(rad), 0]
        T = Transformation.from_frame(Frame([0,0,0], x_vec, y_vec))
        self.robot_frame.transform(T)
        move_base.unadvertise()

    def stop_robot(self):
        self.cmd_vel_clear()
        move_base = self.topic_publish("/robot/cmd_vel", "geometry_msgs/Twist")
        move_base.publish(Message(self.cmd_vel))
        move_base.unadvertise()

    def move_from_frame_to_frame(self, from_frame, to_frame, vel=0.01, linear=True):
        vec = Vector.from_start_end(from_frame.point, to_frame.point)
        rad1 = from_frame.xaxis.angle_signed(vec, [0,0,1])
        rad2 = vec.angle_signed(to_frame.xaxis, [0,0,1])
        self.rotate_in_place(rad1, vel*(rad1/abs(rad1)))
        self.move_forward(vel=vel, dist=vec.length)
        self.rotate_in_place(rad2, vel*(rad2/abs(rad2)))

    def move_to_frame(self, frame, vel=0.01, orient=False):
        vec = Vector.from_start_end(self.robot_frame.point, frame.point)
        rad1 = self.robot_frame.xaxis.angle_signed(vec, [0,0,1])
        rad2 = vec.angle_signed(frame.xaxis, [0,0,1])
        if rad1 > 0:
            self.rotate_in_place(rad1, vel*(rad1/abs(rad1)))
        self.move_forward(vel=vel, dist=vec.length)
        if orient and rad2!=0:
            self.rotate_in_place(rad2, vel*(rad2/abs(rad2)))

    def record_scan(self):
        pass
        

if __name__ == "__main__":
    mb = MobileRobotClient(host='192.168.0.4', port=9090)
    mb.connect()
    time.sleep(1)
    # mb.list_controllers()
    # print(mb.ros.get_nodes())
    # print(mb.ros.get_services())
    # mb.echo_joint_states()
    # config = Configuration()
    # config = Configuration.from_revolute_values([1.57079, 1.57079, 1.57079, 1.57079,1.57079, 1.57079])
    # print(config.joint_values)
    # mb.arm_move_joint(config)
    # config = Configuration.from_revolute_values([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # mb.arm_move_joint(config)
    # mb.set_lift_height(0.0)
    # mb.rotate_in_place()
    mb.move_backward(vel=0.05, dist=0.2)
    # time.sleep(1)
    # mb.move_radial(deg=-90, dist=0.5)
    # frame1 = Frame([0,0,0], [1,0,0], [0,1,0])
    # xvec = [math.cos(math.radians(15)),math.sin(math.radians(15)),0]
    # yvec = [-math.sin(math.radians(15)),math.cos(math.radians(15)),0]
    # frame2 = Frame([0.3,-0.1,0], xvec, yvec)
    # mb.move_from_frame_to_frame(frame1, frame2, vel=0.05)
    # mb.stop_robot()
    # mb.move_forward(vel=0.05, dist=0.2)
    time.sleep(1)
    mb.disconnect()