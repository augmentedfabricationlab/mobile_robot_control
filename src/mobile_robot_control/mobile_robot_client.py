import time
import math
from compas.geometry import Frame, Point, Quaternion, Vector, Transformation
from compas_fab.backends import RosClient
from compas_fab.backends.ros.messages import JointTrajectory, JointTrajectoryPoint, Header
from compas_fab.robots.time_ import Duration
from compas_fab.robots.robot import Configuration
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
    def __init__(self, ros_client=None):
        """_summary_

        Args:
            host (str, optional): IP address of ROS master. Defaults to 'localhost'.
            port (int, optional): Port of ROS master. Defaults to 9090.
        """
        self.ros_client = ros_client
        self.topics = {}
        self.services = {}
        self.tf_clients = {}
        self.action_clients = {}
        
        self.cmd_vel = AttrDict(linear=AttrDict(x=0.0, y=0.0, z=0.0),
                                angular=AttrDict(x=0.0, y=0.0, z=0.0))
        self.tf_frame = None
        
        self.marker_frames = {}
        self.robot_frame = Frame.worldXY()
        
        self.current_joint_values = {}
        
    def connect(self):
        """_summary_
        """
        self.ros_client.run()
        print("Is ROS connected? ", self.ros_client.is_connected)

    def disconnect(self):
        """_summary_
        """
        self.ros_client.close()

    def tf_subscribe(self, target_frame, reference_frame, tf_callback=None):
        if tf_callback == None:
            tf_callback = self._receive_tf_frame_callback
        if not self.tf_clients.get(reference_frame):
            tf_client = tf.TFClient(self.ros_client, fixed_frame=reference_frame, angular_threshold=0.0, rate=10.0)
            self.tf_clients[reference_frame] = tf_client
        else:
            tf_client = self.tf_clients.get(reference_frame)
        tf_client.subscribe(target_frame, tf_callback)

    def tf_unsubscribe(self, target_frame, reference_frame, tf_callback=None):
        if tf_callback == None:
            tf_callback = self._receive_tf_frame_callback
        if self.tf_clients.get(reference_frame):
            tf_client = self.tf_clients.get(reference_frame)
            tf_client.unsubscribe(target_frame, tf_callback)
        
    def _receive_tf_frame_callback(self, message):
        pose_point = Point(message['translation']['x'], message['translation']['y'], message['translation']['z'])
        pose_quaternion = Quaternion(message['rotation']['w'], message['rotation']['x'], message['rotation']['y'], message['rotation']['z'])
        pose_frame = Frame.from_quaternion(pose_quaternion, pose_point)
        self.tf_frame = pose_frame
        
    def clean_tf_frame(self):
        self.tf_frame = None
        
    def service_provide(self, service_name, service_type, handler=None):
        """Start advertising the service. 
        This turns the instance from a client into a server. The callback will be invoked with every request that is made to the service.
        If the service is already advertised, this call does nothing.

        Args:
            service_name (str): Service name. e.g. '/set_ludicrous_speed'
            service_type (str): Sevice type. e.g. 'std_srvs/SetBool'
            handler (func, optional): Callback invoked on every service call. It should accept two parameters: service_request and service_response. It should return True if executed correctly, otherwise False. Defaults to None. 
            e.g. def handler(request, response):
                    print('Setting speed to {}'.format(request['data']))
                    response['success'] = True
                    return True
        """
        if service_name not in self.services.keys():
            self.set_service(service_name, service_type)
        if not self.get_service(service_name).is_advertised:
            self.get_service(service_name).advertise(handler)
            
    def service_unprovide(self, service_name):
        if service_name in self.services.keys():
            if self.get_service(service_name).is_advertised:
                self.get_service(service_name).unadvertise()
            self.remove_service(service_name)
            
    def service_call(self, service_name, service_type, request_dict):
        """Start a service call.

        Args:
            service_name (str): Service name. e.g. '/set_ludicrous_speed'
            service_type (str): Sevice type. e.g. 'std_srvs/SetBool'
            request_dict (dict): Answer to te request as a dictionary. e.g. {'data': True}

        Returns:
            result (dict): Service response.
        """
        if service_name not in self.services.keys():
            self.set_service(service_name, service_type)
        service = self.get_service(service_name)
        request = ServiceRequest(request_dict)
        result = service.call(request)
        return result
            
    def get_service(self, service_name):
        return self.services.get(service_name)
    
    def set_service(self, service_name, service_type):
        self.services[service_name] = Service(self.ros_client, service_name, service_type)
        return self.services[service_name]
    
    def remove_service(self, service_name):
        self.services.pop(service_name)
        
    def is_service_available(self, service_name):
        all_services = self.ros_client.get_services()
        if service_name in all_services:
            return True
        else:
            return False
            
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

    def topic_publish(self, topic_name, msg_type=None, msg_dict=None):
        if topic_name not in self.topics.keys():
            self.set_topic(topic_name, msg_type)
        if not self.topics[topic_name].is_advertised:
            msg = Message(msg_dict)
            self.topics[topic_name].publish(msg)
            
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

    def _receive_joint_states(self, message):
        for key, joint_name in enumerate(message.get('name')):
            self.current_joint_values[joint_name] = message.get('position')[key]
        
    def joint_states_subscribe(self):
        self.topic_subscribe('/robot/joint_states', 'sensor_msgs/JointState', self._receive_joint_states)
    
    def joint_states_unsubscribe(self):
        self.topic_unsubscribe('/robot/joint_states')
        
    def get_current_ur10e_and_liftkit_config(self):
        joint_names_ordered = ['robot_ewellix_lift_top_joint', 'robot_arm_shoulder_pan_joint', 'robot_arm_shoulder_lift_joint', 'robot_arm_elbow_joint', 'robot_arm_wrist_1_joint', 'robot_arm_wrist_2_joint', 'robot_arm_wrist_3_joint']
        joint_values_ordered = [self.current_joint_values.get(joint_name, 0.0) for joint_name in joint_names_ordered]
        joint_types_ordered = [2, 0, 0, 0, 0, 0, 0]
        return Configuration(joint_values_ordered, joint_types_ordered, joint_names_ordered)

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
        self.topic_publish("/robot/lift_joint_position_controller/command", "std_msgs/Float64")
        self.get_topic("/robot/lift_joint_position_controller/command").publish(Message({"data": height}))
        t0 = time.time()
        while time.time()-t0 < 5:
            time.sleep(0.1)
        self.topic_unpublish("/robot/lift_joint_position_controller/command")

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
    # mb.move_backward(vel=0.05, dist=0.2)
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