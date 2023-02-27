from fabrication_manager.task import Task
from ur_fabrication_control.direct_control.fabrication_process import URTask
from ur_fabrication_control.direct_control.mixins import URScript_AreaGrip
from compas.geometry import Frame, Transformation, Translation, Vector, Rotation, Point
from fabrication_manager.communication import TCPFeedbackServer

import time
import math

__all__ = [
    "MobileRobotGetMarkerTask",
    "MobileRobotMoveToMarkerTask"
]

class MobileRobotGetMarkerTask(Task):
    def __init__(self, robot, marker_id="marker_0", reference_frame_id="base", key=None):
        super(MobileRobotGetMarkerTask, self).__init__(key)
        self.robot = robot
        self.marker_id = marker_id
        self.reference_frame_id = reference_frame_id

    def run(self, stop_thread):
        self.robot.mobile_client.clean_tf_frame()
        self.robot.mobile_client.tf_subscribe(self.marker_id, self.reference_frame_id)
        t0 = time.time()
        while time.time() - t0 < 10 and not stop_thread(): #can be used for live subscription when time limit is removed.
            time.sleep(0.1)
            if self.robot.mobile_client.tf_frame is not None:
                self.log('Got the frame: {}'.format(self.robot.mobile_client.tf_frame))
                self.robot.mobile_client.marker_frames[self.marker_id] = self.robot.mobile_client.tf_frame
                break
        self.robot.mobile_client.tf_unsubscribe(self.marker_id, self.reference_frame_id)
        self.is_completed = True
        return True
        
class MobileRobotMoveToMarkerTask(URTask):
    def __init__(self, robot, robot_address, routine=0, marker_id='marker_0', target_marker_id='marker_1', velocity=0.016, radius=0.01, key=None):
        super(MobileRobotMoveToMarkerTask, self).__init__(robot, robot_address, key)
        self.routine = routine
        self.marker_id = marker_id
        self.target_marker_id = target_marker_id
        self.velocity=velocity
        self.radius=radius

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
        
        self.log("Frame to go is: {}".format(frame))
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
        
    def create_urscript_for_pick_place(self, marker_frame_1, marker_frame_2):
        frame_1 = Frame(marker_frame_1.point, marker_frame_1.yaxis, -marker_frame_1.zaxis)
        frame_1_safe = frame_1.transformed(Translation.from_vector(frame_1.xaxis*0.087) * Translation.from_vector(Vector.Zaxis()*0.1))
        
        frame_2 = Frame(marker_frame_2.point, marker_frame_2.yaxis, -marker_frame_2.zaxis)
        frame_2_safe = frame_2.transformed(Translation.from_vector(Vector.Zaxis()*0.15))
        
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
        self.urscript.stop_by_force(10.0)
        self.urscript.end_force_mode()
        
        self.urscript.areagrip_on()
        self.urscript.add_line("\tsleep({})".format(2.0))
        
        self.urscript.set_payload(3.510)
        self.urscript.move_linear(frame_1_safe, self.velocity, self.radius)
        
        self.urscript.move_linear(frame_2_safe, self.velocity, self.radius)
        self.urscript.add_line("\tsleep({})".format(2.0))
        self.urscript.get_force()
        
        self.urscript.force_mode_in_z(10.0, 0.025)
        self.urscript.stop_by_force(10.0)
        self.urscript.end_force_mode()
        
        self.urscript.areagrip_off()
        self.urscript.add_line("\tsleep({})".format(2.0))
        self.urscript.set_payload(1.140)
        
        self.urscript.move_linear(frame_2_safe, self.velocity, self.radius)

        self.urscript.socket_send_line_string(self.req_msg, self.server.name)
        self.urscript.socket_close()
        
        self.urscript.end()
        self.urscript.generate()

    def run(self, stop_thread):
        if self.routine in [0, 1]:
            marker_frame = self.robot.mobile_client.marker_frames[self.marker_id]
            self.create_urscript_for_snapshot_pose(marker_frame)
        else:
            marker_frame_1 = self.robot.mobile_client.marker_frames[self.marker_id]
            marker_frame_2 = self.robot.mobile_client.marker_frames[self.target_marker_id]
            self.create_urscript_for_pick_place(marker_frame_1, marker_frame_2)
        super(MobileRobotMoveToMarkerTask, self).run(stop_thread)
        
if __name__ == "__main__":
    pass