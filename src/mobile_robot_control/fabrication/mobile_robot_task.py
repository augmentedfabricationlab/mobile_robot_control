from fabrication_manager.task import Task
from ur_fabrication_control.direct_control.fabrication_process import URTask
from ur_fabrication_control.direct_control.mixins import URScript_AreaGrip

import time

__all__ = [
    "MobileRobotTaskMarkerSet",
    "MobileRobotTaskMoveToMarker"
]


class MobileRobotTaskMarkerSet(Task):
    def __init__(self, robot, robot_address, marker_type="place", key=None):
        super(MobileRobotTaskMarkerSet, self).__init__(key)
        self.robot = robot
        self.robot_address = robot_address
        self.marker_type = marker_type
        self.target_frame = None
        self.reference_frame = robot.base_frame

    def run(self, stop_thread):
        self.robot.mobile_client.clean_tf_frame()
        self.robot.mobile_client.tf_subscribe(self.target_frame, self.reference_frame)
        t0 = time.time()
        while time.time()-t0<10 and not stop_thread():
            if self.robot.mobile_client.tf_frame is not None:
                if self.marker_type == "place":
                    self.robot.mobile_client.place_frame = self.robot.mobile_client.tf_frame
                elif self.marker_type == "pick":
                    self.robot.mobile_client.pick_frame = self.robot.mobile_client.tf_frame
                break
            else:
                pass        
        self.robot.mobile_client.tf_unsubscribe(self.target_frame, self.reference_frame)
        
class MobileRobotTaskMoveToMarker(URTask):
    def __init__(self, robot, robot_address, routine=0, key=None):
        super(MobileRobotTaskMoveToMarker, self).__init__(key)
        self.robot = robot
        self.robot_address = robot_address
        self.routine = routine

    def create_urscript(self, pick_frame, place_frame=None):
        self.urscript = URScript_AreaGrip()

        self.urscript.socket_open(self.server.ip, self.server.port, self.server.name)
        self.urscript.socket_send_line_string(self.req_msg, self.server.name)
        self.urscript.socket_close()

        self.urscript.end()
        self.urscript.generate()  

    def run(self, stop_thread):
        if self.routine in [0,1,2,3]:
            self.create_urscript(self.robot.mobile_client.pick_frame)
        else:
            self.create_urscript(self.robot.mobile_client.pick_frame,
                                 self.robot.mobile_client.place_frame)
        super(MobileRobotTaskMoveToMarker, self).run()
