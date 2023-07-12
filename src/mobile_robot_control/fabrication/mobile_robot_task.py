from fabrication_manager.task import Task
from ur_fabrication_control.direct_control.fabrication_process import URTask
from ur_fabrication_control.direct_control.mixins import URScript_AreaGrip
from compas_ghpython import draw_frame
from compas_fab.robots import Configuration
from compas.geometry import Frame, Transformation, Point, Vector

import time
import math

__all__ = [
    "MoveJointsTask",
    "MoveLinearTask",
    "MotionPlanTask",
    "SearchMarkersTask",
    "GetMarkerPoseTask",
    "UpdateRobotPoseToMarkerTask"
]
    
class MotionPlanTask(Task):
    def __init__(self, robot, frame_WCF, start_configuration, group, tolerance_position=0.001, tolerance_xaxis=1.0, tolerance_yaxis=1.0, 
                 tolerance_zaxis=1.0, attached_collision_meshes=None, path_constraints=None, planner_id='RRTConnect', recalculate_path=False, key=None):
        super(MotionPlanTask, self).__init__(key)
        self.robot = robot
        self.group = group
        self.frame_WCF = frame_WCF
        self.start_configuration = start_configuration
        self.recalculate_path = recalculate_path
        
        self.tolerance_position = tolerance_position 
        self.tolerance_xaxis = tolerance_xaxis
        self.tolerance_yaxis = tolerance_yaxis 
        self.tolerance_zaxis = tolerance_zaxis
        
        self.path_constraints = path_constraints
        self.attached_collision_meshes = attached_collision_meshes 
        self.planner_id = planner_id
        
        self.trajectory = None
        self.results = {"configurations" : [], "planes" : [], "positions" : [], "velocities" : [], "accelerations" : []}
        
    def run(self, stop_thread):
        tolerances_axes = [math.radians(self.tolerance_xaxis), math.radians(self.tolerance_yaxis), math.radians(self.tolerance_zaxis)]
        frame_BCF = self.frame_WCF.transformed(self.robot.transformation_WCF_BCF())

        if self.robot.attached_tool:
            tool0_BCF = self.robot.from_tcf_to_t0cf([frame_BCF])[0]
        else:
            tool0_BCF = frame_BCF
            
        goal_constraints = self.robot.constraints_from_frame(tool0_BCF, self.tolerance_position, tolerances_axes, self.group)
        
        self.log("Planning trajectory...")
        self.trajectory = self.robot.plan_motion(goal_constraints,
                                    start_configuration=self.start_configuration,
                                    group=self.group,
                                    options=dict(
                                        attached_collision_meshes=self.attached_collision_meshes,
                                        path_constraints=self.path_constraints,
                                        planner_id=self.planner_id,
                                    ))
        
        while not stop_thread():
            if self.trajectory is not None:
                break
            time.sleep(0.1)
        
        self.log('Trajectory found at {}.'.format(self.trajectory))
        
        for c in self.trajectory.points:
            config = self.robot.merge_group_with_full_configuration(c, self.trajectory.start_configuration, self.group)
            joint_names_ordered = ['robot_ewellix_lift_top_joint', 'robot_arm_shoulder_pan_joint', 'robot_arm_shoulder_lift_joint', 'robot_arm_elbow_joint', 'robot_arm_wrist_1_joint', 'robot_arm_wrist_2_joint', 'robot_arm_wrist_3_joint']
            joint_values_ordered = [config.joint_values[config.joint_names.index(joint_name)] for joint_name in joint_names_ordered]
            joint_types_ordered = [config.joint_types[config.joint_names.index(joint_name)] for joint_name in joint_names_ordered]
            mobile_robot_config = Configuration(joint_values_ordered, joint_types_ordered, joint_names_ordered)
            self.results["configurations"].append(mobile_robot_config)
    
            frame_t = self.robot.forward_kinematics(c, self.group, options=dict(solver='model'))
            self.results["planes"].append(draw_frame(frame_t.transformed(self.robot.transformation_BCF_WCF())))
            self.results["positions"].append(c.positions)
            self.results["velocities"].append(c.velocities)
            self.results["accelerations"].append(c.accelerations)
        
        self.is_completed = True
        return True
 
class MoveJointsTask(URTask):
    def __init__(self, robot, robot_address, configuration, velocity=0.10, payload=0.0, key=None):
        super(MoveJointsTask, self).__init__(robot, robot_address, key)
        self.configuration = configuration 
        self.velocity = velocity
        self.payload = payload 

    def create_urscript(self):
        tool_angle_axis = list(self.robot.attached_tool.frame.point) + list(self.robot.attached_tool.frame.axis_angle_vector)
        self.urscript = URScript_AreaGrip(*self.robot_address)
        self.urscript.start()
        self.urscript.set_tcp(tool_angle_axis)
        self.urscript.set_payload(self.payload)
        self.urscript.add_line("textmsg(\">> TASK{}.\")".format(self.key))
        
        self.urscript.set_socket(self.server.ip, self.server.port, self.server.name)
        self.urscript.socket_open(self.server.name)
        
        self.urscript.move_joint(self.configuration, self.velocity)
        
        self.urscript.socket_send_line_string(self.req_msg, self.server.name)
        self.urscript.socket_close(self.server.name)
        
        self.urscript.end()
        self.urscript.generate()
        self.log('Going to set configuration.')
        
    def run(self, stop_thread):
        self.create_urscript()
        super(MoveJointsTask, self).run(stop_thread)
        
class MoveLinearTask(URTask):
    def __init__(self, robot, robot_address, frame_WCF, velocity=0.10, radius=0.0, payload=0.0, key=None):
        super(MoveLinearTask, self).__init__(robot, robot_address, key)
        self.frame_WCF = frame_WCF 
        self.velocity = velocity
        self.radius = radius
        self.payload = payload 

    def create_urscript(self):
        frame_RCF = self.frame_WCF.transformed(self.robot.transformation_WCF_RCF())
    
        tool_angle_axis = list(self.robot.attached_tool.frame.point) + list(self.robot.attached_tool.frame.axis_angle_vector)
        self.urscript = URScript_AreaGrip(*self.robot_address)
        self.urscript.start()
        self.urscript.set_tcp(tool_angle_axis)
        self.urscript.set_payload(self.payload)
        self.urscript.add_line("textmsg(\">> TASK{}.\")".format(self.key))
        
        self.urscript.set_socket(self.server.ip, self.server.port, self.server.name)
        self.urscript.socket_open(self.server.name)
        
        self.urscript.move_linear(frame_RCF, self.velocity, self.radius)
        
        self.urscript.socket_send_line_string(self.req_msg, self.server.name)
        self.urscript.socket_close(self.server.name)
        
        self.urscript.end()
        self.urscript.generate()
        self.log('Going to frame.')
        
    def run(self, stop_thread):
        self.create_urscript()
        super(MoveLinearTask, self).run(stop_thread)

class SearchMarkersTask(Task):
    def __init__(self, robot, robot_address, fabrication, duration=10, key=None):
        super(SearchMarkersTask, self).__init__(key)
        self.robot = robot
        self.robot_address = robot_address
        self.fabrication = fabrication
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
        
        # Iterate the marker ids
        if len(self.marker_ids) > 0:
            for marker_id in self.marker_ids:
                next_key = self.fabrication.get_next_task_key()
                task = GetMarkerPoseTask(self.robot, marker_id=marker_id, reference_frame_id="base", key=next_key)
                self.fabrication.add_task(task, key=next_key)
        else:
            self.log('No more markers are visible.')
            
        self.is_completed = True
        return True
    
class GetMarkerPoseTask(Task):
    def __init__(self, robot, marker_id="marker_0", reference_frame_id="base", key=None):
        super(GetMarkerPoseTask, self).__init__(key)
        self.robot = robot
        self.marker_id = marker_id
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

class UpdateRobotPoseToMarkerTask(Task):
    def __init__(self, robot, fixed_marker_id= "marker_0", key=None):
        super(UpdateRobotPoseToMarkerTask, self).__init__(key)
        self.robot = robot
        self.fixed_marker_id = fixed_marker_id

    def run(self, stop_thread):
        if self.robot.mobile_client.marker_frames.get(self.fixed_marker_id):
            MCF_in_RCF = self.robot.mobile_client.marker_frames[self.fixed_marker_id] # marker frame in RCF
            MCF_in_BCF = MCF_in_RCF.transformed(self.robot.transformation_RCF_BCF()) # marker frame in BCF
            BCF_in_MCF = Frame.from_transformation(Transformation.from_frame(MCF_in_BCF).inverted()) # BCF in measured MCF
            fixed_marker_frame = Frame(Point(0.000, 0.000, 0.000), Vector(0.000, 0.000, 1.000), Vector(0.000, 1.000, -0.000)) # marker frame is rotated in the world
            transformation_FMCF_WCF = Transformation.from_change_of_basis(fixed_marker_frame, Frame.worldXY()) # T from fixed MCF to WCF
            BCF_in_WCF = BCF_in_MCF.transformed(transformation_FMCF_WCF) # BCF in WCF
        
            self.robot.BCF = BCF_in_WCF
            self.log("Robot is fixed to {}.".format(self.fixed_marker_id))
        else:
            self.log("Fixed marker frame is not retrieved.")
        
        self.is_completed = True
        return True
        

if __name__ == "__main__":
    pass