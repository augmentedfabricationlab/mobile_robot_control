from fabrication_manager.task import Task
from ur_fabrication_control.direct_control.fabrication_process import URTask
from ur_fabrication_control.direct_control.mixins import URScript_AreaGrip
from compas.geometry import Frame, Transformation, Translation, Vector, Rotation, Point
from compas_fab.robots import Configuration
from fabrication_manager.communication import TCPFeedbackServer
from assembly_information_model.assembly import Assembly, Element
from compas_fab.robots import CollisionMesh
from compas_ghpython import draw_frame
from compas_rhino.conversions import xform_to_rhino

import time
import math

__all__ = [
    "SearchMarkersTask",
    "GetMarkerPoseTask",
    "HomeConfigurationTask",
    "MarkerSnapshotTask",
    "GetElementPoseTask",
    "PickPlaceElementTask",
    "PlanTrajectoryTask"
]

# class GetJointStates(Task):
#     def __init__(self, robot, assembly_done, key=None):
#         super(GetJointStates, self).__init__(key)
#         self.robot = robot
#         self.assembly_done = assembly_done
#         self.parallelizable = True
        
#     def receive_joint_states(self, message):
#         for key, joint_name in enumerate(message.get('name')):
#             self.robot.ur_joint_values[joint_name] = message.get('position')[key]
                
#     def run(self, stop_thread):
#         #t0 = time.time()
#         #while time.time() - t0 < 10 and not stop_thread():
#         self.log("Subscribing to joint states.")
#         self.robot.mobile_client.topic_subscribe('/joint_states', 'sensor_msgs/JointState', self.receive_joint_states)
        
#         while not self.assembly_done or not stop_thread():
#             time.sleep(0.1)

#         self.log("Unsubscribing from joint states.")
#         self.robot.mobile_client.topic_unsubscribe('/joint_states')
#         self.is_completed = True
#         return True
    
# class EndAllTasks(Task):
#     def __init__(self, assembly_done, key=None):
#         super(EndAllTasks, self).__init__(key)
#         self.assembly_done = assembly_done
#         self.parallelizable = True
        
#     def run(self, stop_thread):
#         self.log("All tasks have ended.")
#         self.assembly_done = True
#         self.is_completed = True
#         return True

class SearchMarkersTask(Task):
    def __init__(self, robot, robot_address, fabrication, element_mesh, duration=10, key=None):
        super(SearchMarkersTask, self).__init__(key)
        self.robot = robot
        self.robot_address = robot_address
        self.fabrication = fabrication
        self.assembly = fabrication.assembly
        self.element_mesh = element_mesh
        self.duration = duration
        self.marker_ids = []
        
    def receive_marker_ids(self, message):
        msg = message.get('transforms')[0]
        if msg.get('header').get('frame_id') == 'camera_color_optical_frame':
            marker_id = msg.get('child_frame_id')
            if marker_id not in self.marker_ids:
                self.log('Found marker with ID: {}'.format(marker_id))
                if marker_id != "marker_1":
                    self.marker_ids.append(marker_id)
                
    def run(self, stop_thread):
        self.marker_ids = []
        
        # Get the marker ids in the scene
        self.robot.mobile_client.topic_subscribe('/tf', 'tf2_msgs/TFMessage', self.receive_marker_ids)
        t0 = time.time()
        while time.time() - t0 < self.duration and not stop_thread():
            time.sleep(0.1)
        self.robot.mobile_client.topic_unsubscribe('/tf')
        self.log('Got all the visible marker ids.')
        time.sleep(1)
        self.log("Length of the list is {}".format(len(self.marker_ids)))
        
        # Iterate the marker ids and create elements
        if len(self.marker_ids) > 0:
            assembly_element_keys = [key for key, element in self.assembly.elements()]
            element_ids = []
            for marker_id in self.marker_ids:
                element_id = int(marker_id.replace("marker_", ""))
                element_ids.append(element_id)
                if element_id not in assembly_element_keys:
                    next_key = self.fabrication.get_next_task_key()
                    task = GetElementPoseTask(self.robot, assembly=self.assembly, element_mesh=self.element_mesh, element_id=element_id, key=next_key)
                    self.fabrication.add_task(task, key=next_key)

            # try:
            #    elements_placed = self.assembly.network.nodes_attribute(element_ids, 'is_placed')
            # except KeyError:
            #     self.log("")
            elements_placed = []
            for element_id in element_ids:
                if self.assembly.network.has_node(element_id):
                    if self.assembly.network.node_attribute(element_id, 'is_placed'):
                        elements_placed.append(True)
                    else:
                        elements_placed.append(False)
            # assembly_done = all([self.assembly.network.node_attribute(element_id, 'is_placed') for element_id in element_ids])
            if len(elements_placed) > 0:
                assembly_done = all(elements_placed)
            else:
                assembly_done = False
            self.log("Assembly is done:".format(assembly_done))
            
            if not assembly_done:
                self.log("Choose element task created.")
                next_key = self.fabrication.get_next_task_key()
                choosing_task = ChooseElementTask(self.robot, self.robot_address, self.fabrication, element_mesh=self.element_mesh, key=next_key)
                self.fabrication.add_task(choosing_task, key=next_key)
            else:
                self.log('No more elements to pick and place.')
        else:
            self.log('No more elements are visible.')
            
        self.is_completed = True
        return True
    
class ChooseElementTask(Task):
    def __init__(self, robot, robot_address, fabrication, element_mesh, key=None):
        super(ChooseElementTask, self).__init__(key)
        self.robot = robot
        self.robot_address = robot_address
        self.fabrication = fabrication
        self.assembly = fabrication.assembly
        self.element_mesh = element_mesh
        
    def run(self, stop_thread):
        self.log('Choosing an element from assembly.')
        
        # Order the element ids by height (highest to lowest)
        element_height_order = []
        for key, element in self.assembly.elements():
            if not self.assembly.network.node_attribute(key, 'is_placed'):
                if element_height_order == []:
                    element_height_order.append(key)
                else:
                    for i in range(len(element_height_order)):
                        if element.frame.point.z > self.assembly.element(element_height_order[i]).frame.point.z:
                            element_height_order.insert(i, key)
                            break
                        elif element.frame.point.z <= self.assembly.element(element_height_order[i]).frame.point.z and i + 1 == len(element_height_order):
                            element_height_order.append(key)
        self.log('Element height order is: {}'.format(element_height_order))
                
        # Iterate the highest order list to pick and place
        for element_id in element_height_order:
            next_key = self.fabrication.get_next_task_key()
            tasks = [MarkerSnapshotTask(self.robot, self.robot_address, routine=0, marker_id=element_id, key=next_key),
                    GetElementPoseTask(self.robot, self.assembly, self.element_mesh, element_id=element_id, key=next_key+1),
                    MarkerSnapshotTask(self.robot, self.robot_address, routine=1, marker_id=element_id, key=next_key+2),
                    GetElementPoseTask(self.robot, self.assembly, self.element_mesh, element_id=element_id, key=next_key+3),
                    PickPlaceElementTask(self.robot, self.robot_address, self.assembly, element_id=element_id, key=next_key+4),
                    GetElementPoseTask(self.robot, self.assembly, self.element_mesh, element_id=element_id, key=next_key+5)]
            self.fabrication.set_tasks(tasks)
            self.log('All pick and place tasks for element {} are created.'.format(element_id))
        
        # Make the initial recursive tasks again.
        next_key = self.fabrication.get_next_task_key()
        self.fabrication.add_task(HomeConfigurationTask(self.robot, self.robot_address, key=next_key), key=next_key)
        self.fabrication.add_task(SearchMarkersTask(self.robot, self.robot_address, self.fabrication, self.element_mesh, key=next_key+1))
        
        self.is_completed = True
        return True

class GetElementPoseTask(Task):
    def __init__(self, robot, assembly, element_mesh, element_id=0, key=None):
        super(GetElementPoseTask, self).__init__(key)
        self.robot = robot
        self.element_id = element_id
        self.marker_id = "marker_" + str(element_id)
        self.reference_frame_id = "base"
        self.assembly = assembly
        self.element_mesh = element_mesh

    def run(self, stop_thread):
        self.log(self.marker_id)
        
        self.robot.mobile_client.clean_tf_frame()
        self.robot.mobile_client.tf_subscribe(self.marker_id, self.reference_frame_id)
        t0 = time.time()
        while time.time() - t0 < 20 and not stop_thread(): # can be used for live subscription when time limit is removed.
            if self.robot.mobile_client.tf_frame is not None:
                self.log('For element {}, got the frame: {}'.format(self.element_id, self.robot.mobile_client.tf_frame))
                self.robot.mobile_client.marker_frames[self.marker_id] = self.robot.mobile_client.tf_frame
                break
            time.sleep(0.1)
        
        if self.robot.mobile_client.tf_frame is None:
            self.log('For element {}, could not get the frame.'.format(self.element_id))
        else:
            keys = [key for key, element in self.assembly.elements()]
            if self.element_id not in keys:
                element = Element(self.robot.mobile_client.tf_frame.copy())
                element.mesh = self.element_mesh
                self.assembly.add_element(element, key=self.element_id, found_frame=self.robot.mobile_client.tf_frame.copy())
                self.log('Created element {}.'.format(self.element_id))
            elif not self.assembly.network.node_attribute(self.element_id, 'is_placed'):
                self.assembly.network.node_attribute(self.element_id, 'found_frame', self.robot.mobile_client.tf_frame.copy())
                self.assembly.element(self.element_id).frame = self.robot.mobile_client.tf_frame.copy()
                self.log('Updated element {} frame.'.format(self.element_id))
            else:
                self.assembly.element(self.element_id).frame = self.robot.mobile_client.tf_frame.copy()
                self.log('Updated element {} frame.'.format(self.element_id))
            # collision_mesh = CollisionMesh(mesh=element.mesh, id=self.element_id, frame=element.frame)
            # self.scene.add_collision_mesh(collision_mesh)
        
        self.robot.mobile_client.tf_unsubscribe(self.marker_id, self.reference_frame_id)
        self.is_completed = True
        
        keys = [key for key, element in self.assembly.elements()]
        self.log("Current element keys: " + str(list(keys)))
        return True

class GetMarkerPoseTask(Task):
    def __init__(self, robot, marker_id=1, reference_frame_id="base", key=None):
        super(GetMarkerPoseTask, self).__init__(key)
        self.robot = robot
        self.marker_id = 'marker_' + str(marker_id)
        self.reference_frame_id = reference_frame_id

    def run(self, stop_thread):
        self.robot.mobile_client.clean_tf_frame()
        self.robot.mobile_client.tf_subscribe(self.marker_id, self.reference_frame_id)
        t0 = time.time()
        while time.time() - t0 < 20 and not stop_thread(): #can be used for live subscription when time limit is removed.
            time.sleep(0.1)
            if self.robot.mobile_client.tf_frame is not None:
                self.log('For {}, got the frame: {}'.format(self.marker_id, self.robot.mobile_client.tf_frame))
                self.robot.mobile_client.marker_frames[self.marker_id] = self.robot.mobile_client.tf_frame
                break
        if self.robot.mobile_client.tf_frame is None:
            self.log('For {}, could not get the frame.'.format(self.marker_id))
        self.robot.mobile_client.tf_unsubscribe(self.marker_id, self.reference_frame_id)
        self.is_completed = True
        return True

class PlanTrajectoryTask(Task):
    def __init__(self, robot, assembly, frames_RCF, element_id=0, recalculate_path=False, key=None):
        super(PlanTrajectoryTask, self).__init__(key)
        self.robot = robot
        self.assembly = assembly
        self.frames_RCF = frames_RCF
        self.element_id = element_id
        self.recalculate_path = recalculate_path
        self.start_configuration = None # put in the GetRobotConfigurationTask() here
        
    def run(self, stop_thread):
        t0 = time.time()
        while time.time() - t0 < 10 and not stop_thread(): 
            time.sleep(0.1)
            trajectory = self.robot.plan_cartesian_motion(self.frames_RCF)
            self.log("This is ")
            self.log(trajectory)
            configurations = trajectory.points
            full_configurations = [self.robot.merge_group_with_full_configuration(c, trajectory.start_configuration, self.robot.main_group_name) for c in configurations]
            frames = [self.robot.forward_kinematics(c, self.robot.main_group_name, options=dict(solver='model')) for c in full_configurations]
            if self.robot.attached_tool:
                frames = [self.robot.from_t0cf_to_tcf([frame])[0] for frame in frames]
            planes = [draw_frame(frame) for frame in frames]
            self.log(planes)
            self.assembly.network.node_attribute(self.element_id, 'planes', planes)

        self.is_completed = True
        return True
    
class PickPlaceElementTask(URTask):
    def __init__(self, robot, robot_address, assembly, element_id=0, velocity=0.10, radius=0.01, key=None):
        super(PickPlaceElementTask, self).__init__(robot, robot_address, key)
        self.assembly = assembly
        self.element_id = element_id
        self.target_marker_id = 'marker_1'
        self.velocity = velocity
        self.radius = radius
        self.number_of_placed_elements = 0
                
    def create_urscript_for_pick_place(self, pick_frame, place_frame):
        self.number_of_placed_elements = len(list(self.assembly.network.nodes_where({'is_placed': True})))
        self.log('{} amount of bricks have been already placed.'.format(self.number_of_placed_elements))
        
        frame_1 = Frame(pick_frame.point, pick_frame.yaxis, -pick_frame.zaxis)
        frame_1_safe = frame_1.transformed(Translation.from_vector(frame_1.xaxis*0.087) * Translation.from_vector(Vector.Zaxis()*0.1))
        
        frame_2 = Frame(place_frame.point, place_frame.yaxis, -place_frame.zaxis) 
        frame_2_safe = frame_2.transformed(Translation.from_vector(-frame_2.yaxis*0.15) * Translation.from_vector(Vector.Zaxis()*(self.number_of_placed_elements*0.05 + 0.1)))
        frame_2_safe_high = frame_2_safe.transformed(Translation.from_vector(Vector.Zaxis()*0.2))
        
        tool_angle_axis = list(self.robot.attached_tool.frame.point) + list(self.robot.attached_tool.frame.axis_angle_vector)
        
        self.urscript = URScript_AreaGrip(*self.robot_address)
        self.urscript.start()
        self.urscript.set_tcp(tool_angle_axis)
        self.urscript.set_payload(1.140)
        self.urscript.add_line("textmsg(\">> TASK{}.\")".format(self.key))
        
        self.urscript.set_socket(self.server.ip, self.server.port, self.server.name)
        self.urscript.socket_open(self.server.name)
        
        self.urscript.move_linear(frame_1_safe, self.velocity, self.radius)
        self.urscript.add_line("\tsleep({})".format(2.0))
        
        self.urscript.get_force()
        
        self.urscript.force_mode_in_z(10.0, 0.025)
        self.urscript.stop_by_force(15.0)
        self.urscript.end_force_mode()
        
        self.urscript.areagrip_on()
        self.urscript.add_line("\tsleep({})".format(2.0))
        
        self.urscript.set_payload(3.510)
        self.urscript.move_linear(frame_1_safe, self.velocity, self.radius)
        
        self.urscript.move_linear(frame_2_safe, self.velocity, self.radius)
        self.urscript.add_line("\tsleep({})".format(2.0))
        self.urscript.get_force()
        
        self.urscript.force_mode_in_z(10.0, 0.025)
        self.urscript.stop_by_force(15.0)
        self.urscript.end_force_mode()
        
        self.urscript.areagrip_off()
        self.urscript.add_line("\tsleep({})".format(2.0))
        self.urscript.set_payload(1.140)
        
        self.urscript.move_linear(frame_2_safe_high, self.velocity, self.radius)
        
        self.urscript.socket_send_line_string(self.req_msg, self.server.name)
        self.urscript.socket_close(self.server.name)
        
        self.urscript.end()
        self.urscript.generate()
        
        self.log('Moving element {} to {}.'.format(self.element_id, self.target_marker_id))

    def run(self, stop_thread):
        pick_frame = self.assembly.element(self.element_id).frame
        place_frame = self.robot.mobile_client.marker_frames[self.target_marker_id]
        self.create_urscript_for_pick_place(pick_frame, place_frame)
        self.assembly.network.node_attribute(self.element_id, 'is_placed', True)
        self.log('Element {} is now placed: {}'.format(self.element_id, self.assembly.network.node_attribute(self.element_id, 'is_placed')))
        super(PickPlaceElementTask, self).run(stop_thread)

class HomeConfigurationTask(URTask):
    def __init__(self, robot, robot_address, velocity=0.10, key=None):
        super(HomeConfigurationTask, self).__init__(robot, robot_address, key)
        self.velocity=velocity

    def create_urscript_for_home_snapshot(self):
        #configuration = Configuration((0.599, -1.031, 1.577, -1.924, -1.607, 1.387), (0, 0, 0, 0, 0, 0))
        configuration = Configuration((0.631, -0.772, 0.942, -1.546, -1.601, 1.419), (0, 0, 0, 0, 0, 0))
        tool_angle_axis = list(self.robot.attached_tool.frame.point) + list(self.robot.attached_tool.frame.axis_angle_vector)
        
        self.urscript = URScript_AreaGrip(*self.robot_address)
        self.urscript.start()
        self.urscript.set_tcp(tool_angle_axis)
        self.urscript.set_payload(1.140)
        self.urscript.add_line("textmsg(\">> TASK{}.\")".format(self.key))
        
        self.urscript.set_socket(self.server.ip, self.server.port, self.server.name)
        self.urscript.socket_open(self.server.name)
        
        self.urscript.move_joint(configuration, self.velocity)
        
        self.urscript.socket_send_line_string(self.req_msg, self.server.name)
        self.urscript.socket_close(self.server.name)
        
        self.urscript.end()
        self.urscript.generate()
        self.log('Going to home configuration.')
        
    def run(self, stop_thread):
        self.create_urscript_for_home_snapshot()
        super(HomeConfigurationTask, self).run(stop_thread)
               
class MarkerSnapshotTask(URTask):
    def __init__(self, robot, robot_address, routine=0, marker_id=0, velocity=0.02, radius=0.01, key=None):
        super(MarkerSnapshotTask, self).__init__(robot, robot_address, key)
        self.routine = routine
        self.marker_id = 'marker_' + str(marker_id)
        self.velocity = velocity
        self.radius = radius

    def create_urscript_for_snapshot_pose(self, marker_frame):
        frame = Frame(marker_frame.point, marker_frame.yaxis, -marker_frame.zaxis)
        if self.routine == 0:
            T1 = Translation.from_vector(Vector.Zaxis() * 0.3)
            R1 = Rotation.from_axis_and_angle(frame.xaxis, math.radians(-20), frame.point)
            R2 = Rotation.from_axis_and_angle(frame.yaxis, math.radians(-20), frame.point)
        else:
            T1 = Translation.from_vector(frame.yaxis * 0.05) * Translation.from_vector(Vector.Zaxis() * 0.1)
            R1 = Rotation.from_axis_and_angle(frame.xaxis, math.radians(20), frame.point)
            R2 = Rotation.from_axis_and_angle(frame.yaxis, math.radians(20), frame.point)
        T = Transformation.concatenated(R2, Transformation.concatenated(R1, T1))
        frame.transform(T)

        tool_angle_axis = list(self.robot.attached_tool.frame.point) + list(self.robot.attached_tool.frame.axis_angle_vector)
        
        self.urscript = URScript_AreaGrip(*self.robot_address)
        self.urscript.start()
        self.urscript.set_tcp(tool_angle_axis)
        self.urscript.set_payload(1.140)
        self.urscript.add_line("textmsg(\">> TASK{}.\")".format(self.key))
        
        self.urscript.set_socket(self.server.ip, self.server.port, self.server.name)
        self.urscript.socket_open(self.server.name)
        
        self.urscript.move_linear(frame, self.velocity, self.radius)
        
        self.urscript.socket_send_line_string(self.req_msg, self.server.name)
        self.urscript.socket_close(self.server.name)
        
        self.urscript.end()
        self.urscript.generate()
        
    def run(self, stop_thread):
        marker_frame = self.robot.mobile_client.marker_frames[self.marker_id]
        self.create_urscript_for_snapshot_pose(marker_frame)
        self.log('Moving to {} snapshot routine {}.'.format(self.marker_id, self.routine))
        super(MarkerSnapshotTask, self).run(stop_thread)
        
if __name__ == "__main__":
    pass