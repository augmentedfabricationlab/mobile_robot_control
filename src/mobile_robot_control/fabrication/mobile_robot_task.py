# from fabrication_manager.task import Task
# from ur_fabrication_control.direct_control.fabrication_process import URTask
# from ur_fabrication_control.direct_control.mixins import URScript_AreaGrip
# from compas_ghpython import draw_frame
# from compas_fab.robots import Configuration
# from compas.geometry import Frame, Transformation, Point, Vector
# import json
# # from pathlib import Path

# import time
# import math

# __all__ = [
#     "MoveJointsTask",
#     "MoveLinearTask",
#     "MotionPlanConfigurationTask",
#     "MotionPlanFrameTask",
#     "InverseKinematicsTask",
#     "GetConfigurationTask",
#     "SearchAndSaveMarkersTask",
#     "GetMarkerPoseTask",
#     "FixRobotToMarkerTask"
# ]

# ### Moveit tasks ###

# class MotionPlanConfigurationTask(Task):
#     def __init__(self, robot, target_configuration, start_configuration, group='ur10e', 
#                  tolerance_above=[math.radians(1)] * 6, tolerance_below=[math.radians(1)] * 6,
#                  attached_collision_meshes=None, path_constraints=None, planner_id='RRTConnect', key=None):
#         super(MotionPlanConfigurationTask, self).__init__(key)
#         self.robot = robot
#         self.group = group
#         self.target_configuration = target_configuration
#         self.start_configuration = start_configuration
        
#         self.tolerance_above = tolerance_above 
#         self.tolerance_below = tolerance_below

#         self.path_constraints = path_constraints
#         self.attached_collision_meshes = attached_collision_meshes 
#         self.planner_id = planner_id
        
#         self.trajectory = None
#         self.results = {"configurations" : [], "planes" : [], "positions" : [], "velocities" : [], "accelerations" : []}

#     def run(self, stop_thread):
#         goal_constraints = self.robot.constraints_from_configuration(
#             configuration=self.target_configuration,
#             tolerances_above=self.tolerance_above,
#             tolerances_below=self.tolerance_below,
#             group=self.group,
#             )

#         self.log("Planning trajectory...")
#         self.trajectory = self.robot.plan_motion(goal_constraints,
#                                     start_configuration=self.start_configuration,
#                                     group=self.group,
#                                     options=dict(
#                                         attached_collision_meshes=self.attached_collision_meshes,
#                                         path_constraints=self.path_constraints,
#                                         planner_id=self.planner_id,
#                                     ))
        
#         while not stop_thread():
#             if self.trajectory is not None:
#                 break
#             time.sleep(0.1)
        
#         self.log('Trajectory found at {}.'.format(self.trajectory))
        
#         for c in self.trajectory.points:
#             config = self.robot.merge_group_with_full_configuration(c, self.trajectory.start_configuration, self.group)
#             joint_names_ordered = ['robot_ewellix_lift_top_joint', 'robot_arm_shoulder_pan_joint', 'robot_arm_shoulder_lift_joint', 'robot_arm_elbow_joint', 'robot_arm_wrist_1_joint', 'robot_arm_wrist_2_joint', 'robot_arm_wrist_3_joint']
#             joint_values_ordered = [config.joint_values[config.joint_names.index(joint_name)] for joint_name in joint_names_ordered]
#             joint_types_ordered = [config.joint_types[config.joint_names.index(joint_name)] for joint_name in joint_names_ordered]
#             mobile_robot_config = Configuration(joint_values_ordered, joint_types_ordered, joint_names_ordered)
#             self.results["configurations"].append(mobile_robot_config)
    
#             frame_t = self.robot.forward_kinematics(c, self.group, options=dict(solver='model'))
#             self.results["planes"].append(draw_frame(frame_t.transformed(self.robot.transformation_BCF_WCF())))
#             self.results["positions"].append(c.positions)
#             self.results["velocities"].append(c.velocities)
#             self.results["accelerations"].append(c.accelerations)
        
#         self.is_completed = True
#         return True

# class MotionPlanFrameTask(Task):
#     def __init__(self, robot, frame_WCF, start_configuration, group='ur10e', 
#                  tolerance_position=0.001, tolerance_xaxis=1.0, tolerance_yaxis=1.0, tolerance_zaxis=1.0, 
#                  attached_collision_meshes=None, path_constraints=None, planner_id='RRTConnect', key=None):
#         super(MotionPlanFrameTask, self).__init__(key)
#         self.robot = robot
#         self.group = group
#         self.frame_WCF = frame_WCF
#         self.start_configuration = start_configuration
        
#         self.tolerance_position = tolerance_position 
#         self.tolerance_xaxis = tolerance_xaxis
#         self.tolerance_yaxis = tolerance_yaxis 
#         self.tolerance_zaxis = tolerance_zaxis
        
#         self.path_constraints = path_constraints
#         self.attached_collision_meshes = attached_collision_meshes 
#         self.planner_id = planner_id
        
#         self.trajectory = None
#         self.results = {"configurations" : [], "planes" : [], "positions" : [], "velocities" : [], "accelerations" : []}

#         self.approved = False
#         self.replan = False
        
#     def run(self, stop_thread):
#         tolerances_axes = [math.radians(self.tolerance_xaxis), math.radians(self.tolerance_yaxis), math.radians(self.tolerance_zaxis)]
#         frame_BCF = self.frame_WCF.transformed(self.robot.transformation_WCF_BCF())
#         tool0_BCF = frame_BCF
#         # if self.robot.attached_tool:
#         #     tool0_BCF = self.robot.from_tcf_to_t0cf([frame_BCF])[0]
#         # else:
#         #     tool0_BCF = frame_BCF
            
#         goal_constraints = self.robot.constraints_from_frame(tool0_BCF, self.tolerance_position, tolerances_axes, self.group)
        
#         while not stop_thread():
#             self.replan = False
#             self.trajectory = None
#             self.results = {"configurations" : [], "planes" : [], "positions" : [], "velocities" : [], "accelerations" : []}

#             self.log("Planning trajectory...")
#             self.trajectory = self.robot.plan_motion(goal_constraints,
#                                         start_configuration=self.start_configuration,
#                                         group=self.group,
#                                         options=dict(
#                                             attached_collision_meshes=self.attached_collision_meshes,
#                                             path_constraints=self.path_constraints,
#                                             planner_id=self.planner_id,
#                                         ))
            
#             while not stop_thread():
#                 if self.trajectory is not None:
#                     break
#                 time.sleep(0.1)
            
#             self.log('Trajectory found at {}.'.format(self.trajectory))
            
#             for c in self.trajectory.points:
#                 config = self.robot.merge_group_with_full_configuration(c, self.trajectory.start_configuration, self.group)
#                 joint_names_ordered = ['robot_ewellix_lift_top_joint', 'robot_arm_shoulder_pan_joint', 'robot_arm_shoulder_lift_joint', 'robot_arm_elbow_joint', 'robot_arm_wrist_1_joint', 'robot_arm_wrist_2_joint', 'robot_arm_wrist_3_joint']
#                 joint_values_ordered = [config.joint_values[config.joint_names.index(joint_name)] for joint_name in joint_names_ordered]
#                 joint_types_ordered = [config.joint_types[config.joint_names.index(joint_name)] for joint_name in joint_names_ordered]
#                 mobile_robot_config = Configuration(joint_values_ordered, joint_types_ordered, joint_names_ordered)
#                 self.results["configurations"].append(mobile_robot_config)
        
#                 frame_t = self.robot.forward_kinematics(c, self.group, options=dict(solver='model'))
#                 self.results["planes"].append(draw_frame(frame_t.transformed(self.robot.transformation_BCF_WCF())))
#                 self.results["positions"].append(c.positions)
#                 self.results["velocities"].append(c.velocities)
#                 self.results["accelerations"].append(c.accelerations)

#             while not stop_thread():
#                 time.sleep(0.1)
#                 if self.approved == True or self.replan == True:
#                     break
#             if self.approved == True:
#                 break
#             time.sleep(0.1)
        
#         self.is_completed = True
#         return True

# class InverseKinematicsTask(Task):
#     def __init__(self, robot, frame_WCF, start_configuration, group='ur10e', json_path=None, key=None):
#         super(InverseKinematicsTask, self).__init__(key)
#         self.robot = robot
#         self.frame_WCF = frame_WCF
#         self.start_configuration = start_configuration
#         self.group = group
#         self.configuration = None
#         self.path = json_path
        
#     def run(self, stop_thread):
#         frame_BCF = self.frame_WCF.transformed(self.robot.transformation_WCF_BCF())
        
#         self.log("Computing inverse kinematics...")
#         self.configuration = self.robot.inverse_kinematics(frame_BCF, self.start_configuration, self.group)
        
#         while not stop_thread():
#             if self.configuration is not None:
#                 break
#             time.sleep(0.1)
        
#         self.log('Configuration found at {}.'.format(self.configuration))
#         # filename = "Task_{}.json".format(self.key)
#         # filepath = self.path / filename
#         # json_data = json.dumps(self.configuration.to_data())

#         # with open(filepath, "w") as f:
#         #     f.write(json_data)
        
#         self.is_completed = True
#         return True

# class GetConfigurationTask(Task):
#     def __init__(self, robot, key=None):
#         super(GetConfigurationTask, self).__init__(key)
#         self.robot = robot
#         self.configuration = None

#     def run(self, stop_thread):
#         self.log("Waiting for current configuration...")
#         current_joint_values = self.robot.mobile_client.current_joint_values

#         joint_names_ordered = ['robot_ewellix_top_lift_joint','robot_arm_shoulder_pan_joint', 'robot_arm_shoulder_lift_joint', 'robot_arm_elbow_joint', 'robot_arm_wrist_1_joint', 'robot_arm_wrist_2_joint', 'robot_arm_wrist_3_joint']
#         joint_values_ordered = [current_joint_values.get(joint_name, 0.00000) for joint_name in joint_names_ordered]
#         joint_types_ordered = [2, 0, 0, 0, 0, 0, 0]
#         self.configuration = Configuration(joint_values_ordered, joint_types_ordered)

#         self.log("Current configuration is: {}".format(self.configuration))
        
#         self.is_completed = True
#         return True
    
# ### UR direct tasks ###

# class MoveJointsTask(URTask):
#     def __init__(self, robot, robot_address, configuration, velocity=0.10, radius=0.1, payload=0.0, CoG=[0.0,0.0,0.0], key=None):
#         super(MoveJointsTask, self).__init__(robot, robot_address, key)
#         self.configuration = configuration 
#         self.velocity = velocity
#         self.radius = radius
#         self.payload = payload
#         self.CoG = CoG

#     def create_urscript(self):
#         if self.robot.attached_tool:
#             tool_angle_axis = list(self.robot.attached_tool.frame.point) + list(self.robot.attached_tool.frame.axis_angle_vector)
#         else:
#             tool0_frame = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])
#             tool_angle_axis = list(tool0_frame.point) + list(tool0_frame.axis_angle_vector)
            
#         self.urscript = URScript_AreaGrip(*self.robot_address)
#         self.urscript.start()
#         self.urscript.set_tcp(tool_angle_axis)
#         self.urscript.set_payload(self.payload, self.CoG)
#         self.urscript.add_line("textmsg(\">> TASK{}.\")".format(self.key))
        
#         self.urscript.set_socket(self.server.ip, self.server.port, self.server.name)
#         self.urscript.socket_open(self.server.name)
        
#         self.urscript.move_joint(self.configuration, self.velocity, self.radius)
        
#         self.urscript.socket_send_line_string(self.req_msg, self.server.name)
#         self.urscript.socket_close(self.server.name)
        
#         self.urscript.end()
#         self.urscript.generate()
#         self.log('Going to set configuration.')
        
#     def run(self, stop_thread):
#         self.create_urscript()
#         super(MoveJointsTask, self).run(stop_thread)
        
# class MoveLinearTask(URTask):
#     def __init__(self, robot, robot_address, frame, in_RCF=True, velocity=0.10, radius=0.0, payload=0.0, key=None):
#         super(MoveLinearTask, self).__init__(robot, robot_address, key)
#         self.robot = robot
#         self.robot_address = robot_address
#         self.frame = frame 
#         self.in_RCF = in_RCF
#         self.velocity = velocity
#         self.radius = radius
#         self.payload = payload 

#     def create_urscript(self):
#         if not self.in_RCF:
#             frame_RCF = self.frame.transformed(self.robot.transformation_WCF_RCF())
#         else:
#             frame_RCF = self.frame
    
#         if self.robot.attached_tool:
#             tool_angle_axis = list(self.robot.attached_tool.frame.point) + list(self.robot.attached_tool.frame.axis_angle_vector)
#         else:
#             tool0_frame = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])
#             tool_angle_axis = list(tool0_frame.point) + list(tool0_frame.axis_angle_vector)
        
#         self.urscript = URScript_AreaGrip(*self.robot_address)
#         self.urscript.start()
#         self.urscript.set_tcp(tool_angle_axis)
#         self.urscript.set_payload(self.payload)
#         self.urscript.add_line("textmsg(\">> TASK{}.\")".format(self.key))
        
#         self.urscript.set_socket(self.server.ip, self.server.port, self.server.name)
#         self.urscript.socket_open(self.server.name)
        
#         self.urscript.move_linear(frame_RCF, self.velocity, self.radius)
        
#         self.urscript.socket_send_line_string(self.req_msg, self.server.name)
#         self.urscript.socket_close(self.server.name)
        
#         self.urscript.end()
#         self.urscript.generate()
#         self.log('Going to frame.')
        
#     def run(self, stop_thread):
#         self.create_urscript()
#         super(MoveLinearTask, self).run(stop_thread)

# ### Marker related tasks ###

# class SearchAndSaveMarkersTask(Task):
#     def __init__(self, robot, robot_address, fabrication, duration=10, update=True, key=None):
#         super(SearchAndSaveMarkersTask, self).__init__(key)
#         self.robot = robot
#         self.robot_address = robot_address
#         self.fabrication = fabrication
#         self.duration = duration
#         self.update = update
#         self.marker_ids = []
        
#     def receive_marker_ids(self, message):
#         msg = message.get('transforms')[0]
#         if msg.get('header').get('frame_id') == 'camera_color_optical_frame':
#             marker_id = msg.get('child_frame_id')
#             if marker_id not in self.marker_ids:
#                 self.log('Found marker with ID: {}'.format(marker_id))
#                 if (self.update) or (not self.update and not self.robot.mobile_client.marker_frames.get(marker_id)):
#                     self.marker_ids.append(marker_id)
#                 else:
#                     self.log("Ignoring {}, as it is already recorded in the marker dictionary and update is set to False.".format(marker_id))
                
#     def run(self, stop_thread):
#         self.marker_ids = []
#         # Get the marker ids in the scene
#         self.robot.mobile_client.topic_subscribe('/tf', 'tf2_msgs/TFMessage', self.receive_marker_ids)
#         t0 = time.time()
#         while time.time() - t0 < self.duration and not stop_thread():
#             time.sleep(0.1)
#         self.robot.mobile_client.topic_unsubscribe('/tf')
#         self.log('Got all the visible marker ids.')
#         time.sleep(1)
#         self.log("Length of the list is {}.".format(len(self.marker_ids)))
        
#         # Iterate the marker ids.
#         if len(self.marker_ids) > 0:
#             for marker_id in self.marker_ids:
#                 next_key = self.fabrication.get_next_task_key()
#                 task = GetMarkerPoseTask(self.robot, marker_id=marker_id, reference_frame_id="robot_arm_base", key=next_key)
#                 self.fabrication.add_task(task, key=next_key)
#         else:
#             self.log('No more markers are visible.')
            
#         self.is_completed = True
#         return True
    
# class SearchAndSaveRobotPoseInMarkerTask(Task):
#     def __init__(self, robot, robot_address, fabrication, duration=10, update=True, key=None):
#         super(SearchAndSaveRobotPoseInMarkerTask, self).__init__(key)
#         self.robot = robot
#         self.robot_address = robot_address
#         self.fabrication = fabrication
#         self.duration = duration
#         self.update = update
#         self.marker_ids = []
        
#     def receive_marker_ids(self, message):
#         msg = message.get('transforms')[0]
#         if msg.get('header').get('frame_id') == 'camera_color_optical_frame':
#             marker_id = msg.get('child_frame_id')
#             if marker_id not in self.marker_ids:
#                 self.log('Found marker with ID: {}'.format(marker_id))
#                 if (self.update) or (not self.update and not self.robot.mobile_client.marker_frames.get(marker_id)):
#                     self.marker_ids.append(marker_id)
#                 else:
#                     self.log("Ignoring {}, as it is already recorded in the marker dictionary and update is set to False.".format(marker_id))
                
#     def run(self, stop_thread):
#         self.marker_ids = []
#         # Get the marker ids in the scene
#         self.robot.mobile_client.topic_subscribe('/tf', 'tf2_msgs/TFMessage', self.receive_marker_ids)
#         t0 = time.time()
#         while time.time() - t0 < self.duration and not stop_thread():
#             time.sleep(0.1)
#         self.robot.mobile_client.topic_unsubscribe('/tf')
#         self.log('Got all the visible marker ids.')
#         time.sleep(1)
#         self.log("Length of the list is {}.".format(len(self.marker_ids)))
        
#         # Iterate the marker ids.
#         if len(self.marker_ids) > 0:
#             for marker_id in self.marker_ids:
#                 next_key = self.fabrication.get_next_task_key()
#                 task = GetRobotPoseInMarkerPoseTask(self.robot, marker_id=marker_id, reference_frame_id="robot_arm_base", key=next_key)
#                 self.fabrication.add_task(task, key=next_key)
#         else:
#             self.log('No more markers are visible.')
            
#         self.is_completed = True
#         return True
    
# class GetRobotPoseInMarkerPoseTask(Task):
#     def __init__(self, robot, marker_id="marker_0", reference_frame_id="robot_arm_base", key=None):
#         super(GetRobotPoseInMarkerPoseTask, self).__init__(key)
#         self.robot = robot
#         self.marker_id = marker_id
#         self.reference_frame_id = reference_frame_id

#     def run(self, stop_thread):
#         self.robot.mobile_client.clean_tf_frame()
#         self.robot.mobile_client.tf_subscribe(self.marker_id, self.reference_frame_id)
#         t0 = time.time()
#         while time.time() - t0 < 20 and not stop_thread(): #can be used for live subscription when time limit is removed.
#             time.sleep(0.1)
#             if self.robot.mobile_client.tf_frame is not None:
#                 MCF_in_RCF = Frame(self.robot.mobile_client.tf_frame.point, self.robot.mobile_client.tf_frame.zaxis, -self.robot.mobile_client.tf_frame.yaxis)
#                 MCF_in_BCF = MCF_in_RCF.transformed(self.robot.transformation_RCF_BCF())
#                 BCF_in_MCF = Frame.from_transformation(Transformation.from_frame(MCF_in_BCF).inverted()) # Invert
#                 self.log('Robot base frame in reference to {} is {}.'.format(self.marker_id, BCF_in_MCF))
                
#                 # Marker frames are added to the dict in WCF.
#                 self.robot.mobile_client.marker_frames[self.marker_id] = BCF_in_MCF
#                 break
#         if self.robot.mobile_client.tf_frame is None:
#             self.log('For {}, could not get the frame.'.format(self.marker_id))
#         self.robot.mobile_client.tf_unsubscribe(self.marker_id, self.reference_frame_id)
#         self.is_completed = True
#         return True 
    
# class GetMarkerPoseTask(Task):
#     def __init__(self, robot, marker_id="marker_0", reference_frame_id="robot_arm_base", key=None):
#         super(GetMarkerPoseTask, self).__init__(key)
#         self.robot = robot
#         self.marker_id = marker_id
#         self.reference_frame_id = reference_frame_id

#     def run(self, stop_thread):
#         self.robot.mobile_client.clean_tf_frame()
#         self.robot.mobile_client.tf_subscribe(self.marker_id, self.reference_frame_id)
#         t0 = time.time()
#         while time.time() - t0 < 20 and not stop_thread(): #can be used for live subscription when time limit is removed.
#             time.sleep(0.1)

#             if self.robot.mobile_client.tf_frame is not None:
#                 MCF_in_RCF = Frame(self.robot.mobile_client.tf_frame.point, self.robot.mobile_client.tf_frame.zaxis, -self.robot.mobile_client.tf_frame.yaxis)
#                 MCF_in_BCF = MCF_in_RCF.transformed(self.robot.transformation_RCF_BCF())
#                 self.log('{} pose in reference to robot base frame is {}.'.format(self.marker_id, MCF_in_BCF))
#                 # Marker frames are added to the dict in WCF.
#                 self.robot.mobile_client.marker_frames[self.marker_id] = MCF_in_BCF
#                 break
#         if self.robot.mobile_client.tf_frame is None:
#             self.log('For {}, could not get the frame.'.format(self.marker_id))
#         self.robot.mobile_client.tf_unsubscribe(self.marker_id, self.reference_frame_id)
#         self.is_completed = True
#         return True

# class FixRobotToMarkerTask(Task):
#     def __init__(self, robot, fixed_marker_id="marker_0", key=None):
#         super(FixRobotToMarkerTask, self).__init__(key)
#         self.robot = robot
#         self.fixed_marker_id = fixed_marker_id
#         self.marker_pose = None

#     def run(self, stop_thread):
#         # Get the frame of the fixed marker id.
#         self.robot.mobile_client.clean_tf_frame()
#         self.robot.mobile_client.tf_subscribe(self.fixed_marker_id, "robot_arm_base")
#         t0 = time.time()
#         while time.time() - t0 < 20 and not stop_thread(): #can be used for live subscription when time limit is removed.
#             time.sleep(0.1)
#             if self.robot.mobile_client.tf_frame is not None:
#                 self.log('For {}, got the frame: {}'.format(self.fixed_marker_id, self.robot.mobile_client.tf_frame))
#                 self.marker_pose = self.robot.mobile_client.tf_frame
#                 break
#         if self.robot.mobile_client.tf_frame is None:
#             self.log('For {}, could not get the frame.'.format(self.fixed_marker_id))
#         self.robot.mobile_client.tf_unsubscribe(self.fixed_marker_id, "robot_arm_base")
        
#         # Fix the robot to the marker pose
#         if self.marker_pose is not None:
#             MCF_in_RCF = self.marker_pose # marker frame in RCF
#             MCF_in_BCF = MCF_in_RCF.transformed(self.robot.transformation_RCF_BCF()) # marker frame in BCF
#             BCF_in_MCF = Frame.from_transformation(Transformation.from_frame(MCF_in_BCF).inverted()) # BCF in measured MCF
#             MCF_in_WCF = self.robot.mobile_client.marker_frames[self.fixed_marker_id] # marker frame in WCF
#             from_MCF_to_WCF = Transformation.from_change_of_basis(MCF_in_WCF, Frame.worldXY()) # T from fixed MCF to WCF
            
#             BCF_in_WCF = BCF_in_MCF.transformed(from_MCF_to_WCF) # BCF in WCF
        
#             self.robot.BCF = BCF_in_WCF
#             self.log("Robot is fixed to {}.".format(self.fixed_marker_id))
#         else:
#             self.log("Fixed marker frame is not retrieved.")
        
#         self.is_completed = True
#         return True
        
# if __name__ == "__main__":
#     pass