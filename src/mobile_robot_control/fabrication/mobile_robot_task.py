from fabrication_manager.task import Task
from ur_fabrication_control.direct_control.fabrication_process import URTask
from ur_fabrication_control.direct_control.mixins import URScript_AreaGrip
from compas_ghpython import draw_frame
from compas_rhino.conversions import xform_to_rhino

import time
import math

__all__ = [
    "FakeTask",
    "MoveJointsTask",
    "MotionPlanTask"
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

class FakeTask(Task):
    def __init__(self, robot, key=None):
        super(FakeTask, self).__init__(key)
        self.robot = robot
        
    def run(self, stop_thread):
        self.log("Fake task text.")
        self.is_completed = True
        
        return True
    
class MotionPlanTask(Task):
    def __init__(self, robot, frame_WCF, start_configuration, group, tolerance_position=0.001, tolerance_xaxis=1.0, tolerance_yaxis=1.0, tolerance_zaxis=1.0, attached_collision_meshes=None, path_constraints=None, planner_id='RRTConnect', recalculate_path=False, key=None):
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
        
        self.results = {"configurations" : [], "planes" : [], "positions" : [], "velocities" : [], "accelerations" : []}
        
    def run(self, stop_thread):
        tolerances_axes = [math.radians(self.tolerance_xaxis), math.radians(self.tolerance_yaxis), math.radians(self.tolerance_zaxis)]
        frame_BCF = self.frame_WCF.transformed(self.robot.transformation_WCF_BCF())
        goal_constraints = self.robot.constraints_from_frame(frame_BCF, self.tolerance_position, tolerances_axes, self.group)
        
        trajectory = None
        trajectory = self.robot.plan_motion(goal_constraints,
                                    start_configuration=self.start_configuration,
                                    group=self.group,
                                    options=dict(
                                        attached_collision_meshes=self.attached_collision_meshes,
                                        path_constraints=self.path_constraints,
                                        planner_id=self.planner_id,
                                    ))

        while not stop_thread(): 
            if trajectory is not None:
                break
            time.sleep(0.1)

        for c in trajectory.points:
            self.results["configurations"].append(self.robot.merge_group_with_full_configuration(c, trajectory.start_configuration, self.group))
            frame = self.robot.forward_kinematics(c, self.group, options=dict(solver='model'))
            self.results["planes"].append(draw_frame(frame.transformed(self.robot.transformation_BCF_WCF())))
            self.results["positions"].append(c.positions)
            self.results["velocities"].append(c.velocities)
            self.results["accelerations"].append(c.accelerations)

        self.log(self.results["planes"])
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
        
if __name__ == "__main__":
    pass