from fabrication_manager.task import Task

__all__ = [
    "MobileRobotTask"
]


class MobileRobotTask(Task):
    def __init__(self, robot, robot_address, key=None):
        super(MobileRobotTask, self).__init__(key)
        self.robot = robot
        self.robot_address = robot_address

    def run(self, stop_thread):
        pass
        # Psuedo code example:
        '''
        self.mobile_base_control.start()
        self.log("Mobile Base Control started...")
        while not self.mobile_base_control.completed:    
            if stop_thread():
                self.log("Forced to stop...")
                self.mobile_base_control.abort()
                break
            else:
                pass
        else:
            self.is_completed = True
            return True
        '''