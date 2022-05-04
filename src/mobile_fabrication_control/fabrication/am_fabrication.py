import sys
from threading import Thread
from time import time

from ur_fabrication_control.direct_control import urscript

from ..extruder_control import ExtruderClient
from ur_fabrication_control.direct_control.fabrication_process import Fabrication
from ur_fabrication_control.direct_control.fabrication_process import FabricationFeedbackServer

if sys.version_info[0] == 3:
    from queue import Queue
else:
    from Queue import Queue

__all__ = [
    "AMFabricationFeedbackServer",
    "AMFabrication"
]


class AMFabricationFeedbackServer(FabricationFeedbackServer):
    def extruder_relay(self, stop, q):
        latest_msg = 0
        ext_msg = None
        while True:
            if stop():
                break
            if ext_msg is not None and q.empty():
                q.put(ext_msg)
            elif not q.unfinished_tasks and ext_msg is not None:
                q.join()
                ext_msg = None
            elif ext_msg is None and len(self.msgs) > latest_msg:
                if "EXTRUDERMSG" in self.msgs[latest_msg]:
                    msg = self.msgs[latest_msg]
                    ext_msg = list(json.loads(msg).values())[0]

    def node_relay(self, stop, q):
        latest_msg = 0
        while True:
            if stop():
                break
            elif len(self.msgs) > latest_msg:
                if 'NODE' in self.msgs[latest_msg]:
                    msg = repr(self.msgs[latest_msg])[1:-1].replace("'", '"')
                    node_msg = json.loads(msg)
                    q.put(node_msg)
                latest_msg += 1


class AMFabrication(Fabrication):
    def __init__(self):
        self.server = None
        self.tasks = {}
        self.stop_task = None
        self._stop_thread = False
        self._performing_task = False
        self.current_task = None

        self._performing_ext_task = False
        self.am_model = None
        self.ext_state = 0
        self.ext_speed = 0
        self.air_state = 0

    def set_extruder_client(self, ip, port):
        self.ec = ExtruderClient(ip, port)

    def set_feedback_server(self, ip, port):
        self.server = AMFabricationFeedbackServer(ip, port)

    def stop_extruder(self):
        if hasattr(self, "ec"):
            self.ext_state = 0
            self.ext_speed = 0
            self.air_state = 0
            self.ec.connect()
            self.ec.send_motordata(0, 17000, 0)
            self.ec.send_set_do(8, 0)
            self.ec.close()

    def relay_to_extruder(self, node):
        if hasattr(self, "ec"):
            if (self.ext_state != node.ext_state
            or self.ext_speed != node.ext_speed):
                self.ec.connect()
                self.ec.send_motordata(node.ext_state, 17000, node.ext_speed)
                self.ext_state = node.ext_state
                self.ext_speed = node.ext_speed
                self.ec.close()
            if self.air_state != node.air_state:
                self.ec.connect()
                self.ec.send_set_do(8, node.air_state)
                self.air_state = node.air_state
                self.ec.close()

    def _join_threads(self):
        self._stop_thread = True
        if hasattr(self, "task_thread"):
            self.task_thread.join()
            del self.task_thread
        if hasattr(self, "listen_thread"):
            self.listen_thread.join()
            del self.listen_thread
        if hasattr(self, "relay_thread"):
            self.relay_thread.join()
            del self.relay_thread

    def _create_threads(self):
        self._stop_thread = False
        self.ur_q = Queue()
        self.node_q = Queue()
        self.task_thread = Thread(target=self.run,
                                  args=(lambda: self._stop_thread,
                                        self.ur_q, self.node_q))
        self.listen_thread = Thread(target=self.server.listen,
                                    args=(lambda: self._stop_thread,
                                          self.ur_q))
        self.relay_thread = Thread(target=self.server.node_relay,
                                   args=(lambda: self._stop_thread,
                                         self.node_q))
        self.task_thread.daemon = True
        self.listen_thread.daemon = True
        self.relay_thread.daemon = True

    def start(self):
        self._join_threads()
        if self.tasks_available():
            self.server.start()
            self.server.clear()
            self._create_threads()
            self.listen_thread.start()
            print("Started listening thread")
            self.relay_thread.start()
            print("Started relay thread")
            self.task_thread.start()
            print("Started task thread")
        else:
            print("No_tasks_available")

    def stop(self):
        self.stop_extruder()
        self.perform_task(self.stop_task)
        self._join_threads()

    def close(self):
        self._join_threads()
        self.server.clear()
        self.server.shutdown()
        self.ec.close()

    def run(self, stop_thread, ur_q, node_q):
        while self.tasks_available():
            if stop_thread():
                self._performing_task = False
                self._performing_ext_task = False
                break
            else:
                # UR communication
                if not self._performing_task:
                    self.current_task = self.get_next_task()
                    ur_q.put(self.tasks[self.current_task]["exit_message"])
                    self.perform_task(self.tasks[self.current_task]['task'])
                    self._performing_task = True
                elif self._performing_task and not ur_q.unfinished_tasks:
                    ur_q.join()
                    print("joined task {}".format(self.current_task))
                    self.tasks[self.current_task]["state"] = "completed"
                    self._performing_task = False

                # Extruder communication
                if not self._performing_ext_task and node_q.unfinished_tasks:
                    self.node_msg = node_q.get()
                    print(self.node_msg)
                    self._performing_ext_task = True
                elif self._performing_ext_task:
                    layer = self.am_model.layer(self.node_msg["LAYER"])
                    node = layer.node(self.node_msg["NODE"])
                    node.is_constructed = True
                    self.relay_to_extruder(node)
                    node_q.task_done()
                    self._performing_ext_task = False


if __name__ == "__main__":
    import json
    import time
    from compas.geometry import Frame
    from am_information_model import AMModel, Layer, Node
    from mobile_fabrication_control.extruder_control.urscript_extrusion import URScript_Extrusion

    ur_ip = "192.168.56.102"
    ur_port = 30002
    server_ip = "192.168.56.101"
    server_port = 50003

    am_model = AMModel()
    layer = Layer()
    for i in range(15):
        new_frame = Frame([-1+(0.1*i),0.5,0], [1,0,0], [0,-1,0])
        new_node = Node(new_frame, 0.005)
        new_node.ext_speed = 100*i
        new_node.ext_state = (i!=14)
        new_node.air_state = (i!=14)
        layer.add_node(new_node)
    am_model.add_layer(layer)

    ur_script = URScript_Extrusion(ur_ip=ur_ip, ur_port=ur_port)
    ur_script.start()
    ur_script.socket_open(server_ip, server_port, "Feedbackserver")
    for key, node in layer.nodes():
        node_msg = {"LAYER":0, "NODE":key}
        ur_script.socket_send_line_string(str(node_msg), socket_name="Feedbackserver")
        ur_script.add_line("sleep(0.5)")
    ur_script.socket_send_line_string("Layer0_completed", socket_name="Feedbackserver")
    ur_script.socket_close(name="Feedbackserver")
    ur_script.end()
    ur_script.generate()
    
    print(ur_script.script)

    stop_script = URScript_Extrusion(ur_ip=ur_ip, ur_port=ur_port)
    stop_script.start()
    stop_script.end()
    stop_script.generate()

    fabrication = AMFabrication()
    fabrication.set_feedback_server(server_ip, server_port)
    fabrication.am_model = am_model.copy()
    fabrication.add_task(ur_script, "Layer0_completed")
    fabrication.stop_task = stop_script
    
    fabrication.start()

    k = 0
    while k<1000:
        time.sleep(0.1)
        k+=1
    
    fabrication.stop()
    print(fabrication.server.msgs)
