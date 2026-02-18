from fabrication_manager.task import Task

from ur_fabrication_control.direct_control.fabrication import URTask
from ur_fabrication_control.direct_control.mixins import URScript, URScript_AreaGrip, URScript_ParallelGrip, URScript_Drill

from compas_rhino.conversions import frame_to_rhino_plane
from compas_robots import Configuration
from compas.geometry import Frame, Transformation, Point, Vector, Translation, Rotation
import json

import time
import math
import roslibpy

__all__ = [
    "SendNavigationActionTask",
    "MoveMobileBaseTask",
    "MotionPlanExecutePose",
    "MotionPlanExecuteJoints",
    "AddCollisionMeshes",
    "RemoveCollisionMesh",
    "ChangeToolFrameTask",
    "SimGripperControlTask",
    "DrillBrickURTask",
    "PickBrickURTask",
    "PlaceBrickURTask",
    "BreakBrickURTask",
    "MoveJointsURdirectTask",
    "MoveLinearURdirectTask",
    "MotionPlanConfigurationTask",
    "MotionPlanFrameTask",
    "InverseKinematicsTask",
    "GetConfigurationTask",
    "ExecuteMotionTask",
    "SearchAndSaveMarkersTask",
    "GetMarkerPoseTask",
    "FixRobotToMarkerTask",
]

### Move Robot base tasks ###

# Via navigation action for Nav2
class SendNavigationActionTask(Task):
    def __init__(self, robot, target_frame = None, key=None):
        super(SendNavigationActionTask, self).__init__(key)
        self.robot = robot
        self.target_frame = target_frame
        self.result = None
        self.done = False
        self.success = False
        
    def _result_callback(self, msg):
        self.result = msg
        self.done = True
        self.success = True
        
    def _feedback_callback(self, msg):
        self.log(f"Distance remaining: {msg['distance_remaining']}")

    def _fail_callback(self, msg):
        self.result = msg
        self.done = True
        self.success = False
        
    def run(self, stop_thread):
        self.log("Sending target frame as navigation action.")

        action_name = '/robot/navigate_to_pose'
        if action_name not in self.robot.mobile_client.action_clients.keys():
            action_client = roslibpy.ActionClient(
                self.robot.mobile_client.ros_client,
                action_name,
                'nav2_msgs/action/NavigateToPose'
            )
        else:
            action_client = self.robot.mobile_client.action_clients[action_name]

        # Create the goal message
        goal = roslibpy.Goal(
            {
                'pose': {
                    'header': {
                        'frame_id': 'robot_map',
                        'stamp': {
                            'sec': int(time.time()),
                            'nanosec': 0
                        }
                    },
                    'pose': {
                        'position': {'x': 4.5, 'y': 0.0, 'z': 0.0},
                        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                    }
                },
                'behavior_tree': ''
            }
        )

        self.log('Sending goal...')
        goal_id = action_client.send_goal(goal, self._result_callback, self._feedback_callback, self._fail_callback)

        while not stop_thread():
            if self.success or self.done:
                break
            time.sleep(0.1)
        
        self.log(f"Action result: {self.result}")

        self.is_completed = True
        return True

# Via direct wheel commands for swerve drive
class MoveMobileBaseTask(Task):
    def __init__(self, robot, velocity=1.0, right=True, linear=True, total_time=7, key=None):
        super(MoveMobileBaseTask, self).__init__(key)
        self.robot = robot
        self.linear = linear
        self.velocity = velocity
        self.right = right
        self.total_time = total_time
        # robot dimensions (meters)
        self.L = 0.959   # front-back - wheelbase
        self.W = 0.585   # left-right -trackwidth
    
    def compute_swerve_rotation(self, L, W, omega, x_p=0, y_p=0):
        """
        Compute wheel steering angles and wheel speeds
        for rotation about robot center.

        L: front-back distance
        W: left-right distance
        omega: angular velocity (rad/s)
        """

        # Wheel positions
        wheels = {
            "back_left":   (-L/2,  W/2),
            "front_left":  ( L/2,  W/2),
            "back_right":  (-L/2, -W/2),
            "front_right": ( L/2, -W/2),
        }

        angles = []
        speeds = []

        for (x, y) in wheels.values():
            # Velocity due to rotation
            dx = x - x_p
            dy = y - y_p

            v_x = -omega * dy
            v_y =  omega * dx


            angle = math.atan2(v_y, v_x)
            speed = math.sqrt(v_x**2 + v_y**2)

            angles.append(angle)
            speeds.append(speed)

        return angles, speeds

    def run(self, stop_thread):
        msg = {
            'data' : False
        }

        while not stop_thread():
            if self.robot.mobile_client.is_service_available('/fix_robot_service'):
                result = self.robot.mobile_client.service_call('/fix_robot_service', 'std_srvs/srv/SetBool', msg)
                success = result['success']
                message = result['message']
                self.log('Message: {}'.format(message))
                if success:
                    break
            else:
                self.log("Fix robot service is not available.")
                break
            time.sleep(0.1)
        
        if self.linear:
            if self.right:
                angle = math.radians(-90)
            else:
                angle = math.radians(90)
            position = [angle, angle, angle, angle]
            velocity = [self.velocity, self.velocity, self.velocity, self.velocity]
        else:
            if self.right:
                omega = self.velocity  # rad/s (positive = CCW)
            else:
                omega = -self.velocity
            position, velocity = self.compute_swerve_rotation(self.L, self.W, omega, x_p=2)

        client = self.robot.mobile_client.ros_client

        publisher = roslibpy.Topic(client, '/base_joint_commands', 'sensor_msgs/JointState')
        joint_names = ["robot_back_left_motor_wheel_joint", "robot_front_left_motor_wheel_joint", 
                         "robot_back_right_motor_wheel_joint",  "robot_front_right_motor_wheel_joint",]
        
        self.log("Position: {}, Velocity: {}, Total time: {}".format(str(position), str(velocity), str(self.total_time)))
        
        while not stop_thread():
            msg = {
                'header': {'stamp': {'secs': int(time.time()), 'nsecs': int((time.time() % 1) * 1e9)}},
                'name': joint_names,
                'position': position,
                'velocity': [0,0,0,0],
                }
            publisher.publish(msg)
            time.sleep(2)

            msg = {
                    'header': {'stamp': {'secs': int(time.time()), 'nsecs': int((time.time() % 1) * 1e9)}},
                    'name': joint_names,
                    'position': position,
                    'velocity': velocity
                }
            publisher.publish(msg)
            total_time = self.total_time #15 #.5m ->7 #6m -> 70 #1m -> 15 # 3m -> 40 # 0.4 -> 5
            time.sleep(total_time)

            msg = {
                'header': {'stamp': {'secs': int(time.time()), 'nsecs': int((time.time() % 1) * 1e9)}},
                'name': joint_names,
                'position': [0,0,0,0],
                'velocity': [0,0,0,0],
                }
            publisher.publish(msg)
            time.sleep(1)
            break
        
        msg = {
            'data' : True
        }
        while not stop_thread():
            if self.robot.mobile_client.is_service_available('/fix_robot_service'):
                result = self.robot.mobile_client.service_call('/fix_robot_service', 'std_srvs/srv/SetBool', msg)
                success = result['success']
                message = result['message']
                self.log('Message: {}'.format(message))
                if success:
                    break
            else:
                self.log("Fix robot service is not available.")
                break
            time.sleep(0.1)
            
        publisher.unadvertise()
        self.log("Arrived at target base frame.")
        
        self.is_completed = True
        return True

### Motion plan tasks via moveit_py services ###

class MotionPlanExecutePose(Task):
    def __init__(self, robot, target_frame=None, frame_in_WCF=True, group="ur20", ee_link="robot_arm_tool0", execute=True, tool_constraint=False, key=None):
        super(MotionPlanExecutePose, self).__init__(key)
        self.robot = robot
        self.target_frame = target_frame
        self.frame_in_WCF = frame_in_WCF
        self.group = group
        self.ee_link = ee_link
        self.execute = execute
        self.tool_constraint = tool_constraint
    
    def run(self, stop_thread):
        if self.robot.attached_tool:
            self.target_frame = self.robot.from_tcf_to_t0cf([self.target_frame])[0]
            self.log("Attached tool.")

        if self.frame_in_WCF:
            frame_BCF = self.robot.from_WCF_to_BCF(self.target_frame)
        else:
            frame_BCF = self.target_frame

        if self.tool_constraint:
            path_constraints = {
            'name': 'keep_tool_in_xy_plane',
            'joint_constraints': [],
            'position_constraints': [],
            'orientation_constraints': [
                {
                    'header': {'frame_id': 'robot_base_footprint'},           # reference frame
                    'orientation': {'x': -0.705, 'y': 0.709, 'z': 0.0, 'w': 0.0},  # tool facing downward
                    'link_name': 'robot_arm_tool0',                # your EE link
                    'absolute_x_axis_tolerance': 3,              # allow slight tilt
                    'absolute_y_axis_tolerance': 3,
                    'absolute_z_axis_tolerance': 6.283,            # free rotation around Z
                    'weight': 1.0
                }
            ],
            'visibility_constraints': []
        }
        else:
            path_constraints = {
                'name': '',
                'joint_constraints': [],
                'position_constraints': [],
                'orientation_constraints': [],
                'visibility_constraints': []
            }

        msg = {
            'pose' : {
                'header' : {'frame_id' : 'robot_base_footprint'},
                'pose' : {
                    'position' : {
                            'x' : frame_BCF.point.x,
                            'y' : frame_BCF.point.y,
                            'z' : frame_BCF.point.z},
                    'orientation' : {
                            'x' : frame_BCF.quaternion.x,
                            'y' : frame_BCF.quaternion.y,
                            'z' : frame_BCF.quaternion.z,
                            'w' : frame_BCF.quaternion.w}
                }
            },
            'group' : self.group,
            'ee_link' : self.ee_link,
            'execute' : self.execute,
            'path_constraints': path_constraints
        }
        # self.log(msg)

        while not stop_thread():
            if self.robot.mobile_client.is_service_available('/moveit/plan_execute_pose'):
                self.log("Motion planning for pose service is available.")
                result = self.robot.mobile_client.service_call('/moveit/plan_execute_pose', 'moveit_ros_interface/srv/PlanExecutePose', msg)
                success = result['success']
                message = result['message']
                trajectory = result['trajectory']
                # self.log('Trajectory: {}'.format(trajectory))
                if success:
                    break
            else:
                self.log("Motion planning for pose service is not available.")
                break
            time.sleep(0.1)
        
        time.sleep(0.1)
        self.is_completed = True
        return True

class MotionPlanExecuteJoints(Task):
    def __init__(self, robot, configuration=None, group="ur20", execute=True, key=None):
        super(MotionPlanExecuteJoints, self).__init__(key)
        self.robot = robot
        self.configuration = configuration
        self.group = group
        self.execute = execute
    
    def run(self, stop_thread):
        msg = {
            'group' : self.group,
            'names' : self.configuration.joint_names,
            'positions' : self.configuration.joint_values,
            'execute' : self.execute
        }
        self.log(msg)

        while not stop_thread():
            if self.robot.mobile_client.is_service_available('/moveit/plan_execute_joints'):
                self.log("Motion planning for joints service is available.")
                result = self.robot.mobile_client.service_call('/moveit/plan_execute_joints', 'moveit_ros_interface/srv/PlanExecuteJoints', msg)
                success = result['success']
                message = result['message']
                trajectory = result['trajectory']
                #self.log('Trajectory: {}'.format(trajectory))
                if success:
                    break
            else:
                self.log("Motion planning for joints service is not available.")
                break
            time.sleep(0.1)
        
        time.sleep(1)
        self.is_completed = True
        return True

class AddCollisionMeshes(Task):
    def __init__(self, robot, mesh, frame=Frame.worldXY(), mesh_id="tool_box", attached=True, key=None):
        super(AddCollisionMeshes, self).__init__(key)
        self.robot = robot
        # Convert inputs to lists if they're not already
        self.meshes = [mesh] if not isinstance(mesh, list) else mesh
        self.frames = [frame] if not isinstance(frame, list) else frame
        self.mesh_ids = [mesh_id] if not isinstance(mesh_id, list) else mesh_id
        # Validate list lengths
        if not (len(self.meshes) == len(self.frames) == len(self.mesh_ids)):
            raise ValueError("mesh, frame, and mesh_id must have the same length")
        self.attached = attached

    def compas_mesh_to_ros(self, mesh):
        """
        Convert COMPAS mesh vertices/faces to ROS Mesh format.
        - vertices: list of [x,y,z]
        - faces: list of [i,j,k]
        """
        vertices = [mesh.vertex_coordinates(i) for i in mesh.vertices()]
        faces = [mesh.face_vertices(f) for f in mesh.faces()]

        ros_vertices = []
        ros_triangles = []
        for v in vertices:
            ros_vertices.append({
                'x': float(v[0]),
                'y': float(v[1]),
                'z': float(v[2])
            })
        for f in faces:
            if len(f) == 3:
                ros_triangles.append({'vertex_indices': f})
            elif len(f) == 4:
                ros_triangles.append({'vertex_indices': [f[0], f[1], f[2]]})
                ros_triangles.append({'vertex_indices': [f[0], f[2], f[3]]})
            else:
                raise ValueError(f"Face with {len(f)} vertices not supported")
        return ros_vertices, ros_triangles

    def run(self, stop_thread):
        if self.attached:
            operation = "ATTACH"
            link_name = 'robot_arm_tool0'
        else:
            operation = "ADD"
            link_name = ''

        for mesh, frame, mesh_id in zip(self.meshes, self.frames, self.mesh_ids):
            vertices, triangles = self.compas_mesh_to_ros(mesh)
            pose = {
                'position': {
                    'x': frame.point.x,
                    'y': frame.point.y,
                    'z': frame.point.z
                },
                'orientation': {
                    'x': frame.quaternion.x,
                    'y': frame.quaternion.y,
                    'z': frame.quaternion.z,
                    'w': frame.quaternion.w
                }
            }
            msg = {
                'id': mesh_id,
                'operation': operation,
                'link_name': link_name,
                'vertices': vertices,
                'triangles': triangles,
                'pose': pose
            }

            self.log(f"Processing mesh {mesh_id}")
            while not stop_thread():
                if self.robot.mobile_client.is_service_available('/moveit/collision_mesh'):
                    result = self.robot.mobile_client.service_call(
                        '/moveit/collision_mesh',
                        'moveit_ros_interface/srv/CollisionMeshOp', 
                        msg
                    )
                    if result['success']:
                        break
                    self.log(f"Failed to add mesh {mesh_id}: {result['message']}")
                time.sleep(0.1)

        time.sleep(1)
        self.is_completed = True
        return True

class RemoveCollisionMesh(Task):
    def __init__(self, robot, frame=Frame.worldXY(), mesh_id="tool_box", key=None):
        super(RemoveCollisionMesh, self).__init__(key)
        self.robot = robot
        self.mesh_id = mesh_id
        self.frame = frame

    def run(self, stop_thread):
        pose = {'position' : {
                            'x' : self.frame.point.x,
                            'y' : self.frame.point.y,
                            'z' : self.frame.point.z},
                    'orientation' : {
                            'x' : self.frame.quaternion.x,
                            'y' : self.frame.quaternion.y,
                            'z' : self.frame.quaternion.z,
                            'w' : self.frame.quaternion.w}
                }
        msg = {
            'id': self.mesh_id,
            'operation': 'REMOVE',
            'link_name': '',      # not needed for REMOVE
            'vertices': [],       # not needed for REMOVE
            'triangles': [],      # not needed for REMOVE
            'pose': pose
        }
        self.log(f"Requesting removal of collision mesh: {self.mesh_id}")
        while not stop_thread():
            if self.robot.mobile_client.is_service_available('/moveit/collision_mesh'):
                self.log("Collision mesh service is available.")
                result = self.robot.mobile_client.service_call(
                    '/moveit/collision_mesh',
                    'moveit_ros_interface/srv/CollisionMeshOp',
                    msg
                )
                success = result['success']
                message = result['message']
                self.log(f"Service response: {success} - {message}")
                break
            else:
                self.log("Collision mesh service is not available.")
                break
            time.sleep(0.2)
        self.is_completed = True
        return True

### Motion plan tasks via compas ###

class MotionPlanConfigurationTask(Task):
    def __init__(
        self,
        robot,
        target_configuration,
        start_configuration,
        group="ur20",
        tolerance_above=[math.radians(1)] * 6,
        tolerance_below=[math.radians(1)] * 6,
        attached_collision_meshes=None,
        path_constraints=None,
        planner_id="RRTConnect",
        validation=True,
        key=None,
    ):
        super(MotionPlanConfigurationTask, self).__init__(key)
        self.robot = robot
        self.group = group
        self.target_configuration = target_configuration
        self.start_configuration = start_configuration

        self.tolerance_above = tolerance_above
        self.tolerance_below = tolerance_below

        self.path_constraints = path_constraints
        self.attached_collision_meshes = attached_collision_meshes
        self.planner_id = planner_id

        self.trajectory = None
        self.results = {
            "configurations": [],
            "planes": [],
            "positions": [],
            "velocities": [],
            "accelerations": [],
        }

        self.validation = validation
        self.replan = False
        self.approved = False

    def run(self, stop_thread):
        goal_constraints = self.robot.constraints_from_configuration(self.target_configuration, self.tolerance_above, self.tolerance_below, self.group)

        self.log("Planning trajectory...")

        while not stop_thread():
            # Clean trajectory.
            self.replan = False
            self.trajectory = None
            self.results = {
                "configurations": [],
                "planes": [],
                "positions": [],
                "velocities": [],
                "accelerations": [],
                }

            self.trajectory = self.robot.plan_motion(
                goal_constraints,
                start_configuration=self.start_configuration,
                group=self.group,
                options=dict(
                    attached_collision_meshes=self.attached_collision_meshes,
                    path_constraints=self.path_constraints,
                    planner_id=self.planner_id,
                ),
            )

            while not stop_thread():
                if self.trajectory is not None:
                    break
                time.sleep(0.1)

            self.log("Trajectory found at {}.".format(self.trajectory))

            for c in self.trajectory.points:
                config = self.robot.merge_group_with_full_configuration(
                    c, self.trajectory.start_configuration, self.group
                )
                joint_names_ordered = [
                    "robot_ewellix_lift_top_joint",
                    "robot_arm_shoulder_pan_joint",
                    "robot_arm_shoulder_lift_joint",
                    "robot_arm_elbow_joint",
                    "robot_arm_wrist_1_joint",
                    "robot_arm_wrist_2_joint",
                    "robot_arm_wrist_3_joint",
                ]
                joint_values_ordered = [
                    config.joint_values[config.joint_names.index(joint_name)]
                    for joint_name in joint_names_ordered
                ]
                joint_types_ordered = [
                    config.joint_types[config.joint_names.index(joint_name)]
                    for joint_name in joint_names_ordered
                ]
                mobile_robot_config = Configuration(
                    joint_values_ordered, joint_types_ordered, joint_names_ordered
                )
                self.results["configurations"].append(mobile_robot_config)

                frame_t = self.robot.forward_kinematics(
                    c, self.group, options=dict(solver="model")
                )
                self.results["planes"].append(
                    frame_to_rhino_plane(
                        frame_t.transformed(self.robot.transformation_BCF_WCF())
                    )
                )
                self.results["positions"].append(c.positions)
                self.results["velocities"].append(c.velocities)
                self.results["accelerations"].append(c.accelerations)

            if self.validation:
                # Wait until trajectory is approved or replan is requested.
                while not stop_thread():
                    time.sleep(0.1)
                    if self.approved == True or self.replan == True:
                        break
                # Break if trajectory is approved.
                if self.approved == True:
                    break
                time.sleep(0.1)
            else:
                break
                
        self.is_completed = True
        return True

class MotionPlanFrameTask(Task):
    def __init__(
        self,
        robot,
        frame_WCF,
        start_configuration,
        group="ur20",
        tolerance_position=0.001,
        tolerance_xaxis=1.0,
        tolerance_yaxis=1.0,
        tolerance_zaxis=1.0,
        attached_collision_meshes=None,
        path_constraints=None,
        planner_id="RRTConnect",
        validation=True,
        key=None,
    ):
        super(MotionPlanFrameTask, self).__init__(key)
        self.robot = robot
        self.group = group
        self.frame_WCF = frame_WCF
        self.start_configuration = start_configuration

        self.tolerance_position = tolerance_position
        self.tolerances_axes = [
                math.radians(tolerance_xaxis),
                math.radians(tolerance_yaxis),
                math.radians(tolerance_zaxis),
            ]

        self.path_constraints = path_constraints
        self.attached_collision_meshes = attached_collision_meshes
        self.planner_id = planner_id

        self.trajectory = None
        self.results = {
            "configurations": [],
            "planes": [],
            "positions": [],
            "velocities": [],
            "accelerations": [],
        }

        self.validation = validation
        self.replan = False
        self.approved = False

    def run(self, stop_thread):
        
        frame_BCF = self.robot.from_WCF_to_BCF(self.frame_WCF)
        goal_constraints = self.robot.constraints_from_frame(
            frame_BCF, 
            self.tolerance_position, 
            self.tolerances_axes, 
            self.group)

        self.log("Planning trajectory...")
        
        while not stop_thread():
            # Clean trajectory.
            self.replan = False
            self.trajectory = None
            self.results = {
                "configurations": [],
                "planes": [],
                "positions": [],
                "velocities": [],
                "accelerations": [],
                }

            self.trajectory = self.robot.plan_motion(
                goal_constraints,
                start_configuration=self.start_configuration,
                group=self.group,
                options=dict(
                    attached_collision_meshes=self.attached_collision_meshes,
                    path_constraints=self.path_constraints,
                    planner_id=self.planner_id,
                ),
            )

            while not stop_thread():
                if self.trajectory is not None:
                    break
                time.sleep(0.1)

            self.log("Trajectory found at {}.".format(self.trajectory))

            for c in self.trajectory.points:
                config = self.robot.merge_group_with_full_configuration(
                    c, self.trajectory.start_configuration, self.group
                )
                joint_names_ordered = [
                    "robot_ewellix_lift_top_joint",
                    "robot_arm_shoulder_pan_joint",
                    "robot_arm_shoulder_lift_joint",
                    "robot_arm_elbow_joint",
                    "robot_arm_wrist_1_joint",
                    "robot_arm_wrist_2_joint",
                    "robot_arm_wrist_3_joint",
                ]
                joint_values_ordered = [
                    config.joint_values[config.joint_names.index(joint_name)]
                    for joint_name in joint_names_ordered
                ]
                joint_types_ordered = [
                    config.joint_types[config.joint_names.index(joint_name)]
                    for joint_name in joint_names_ordered
                ]
                mobile_robot_config = Configuration(
                    joint_values_ordered, joint_types_ordered, joint_names_ordered
                )
                self.results["configurations"].append(mobile_robot_config)

                frame_t = self.robot.forward_kinematics(
                    c, self.group, options=dict(solver="model")
                )
                self.results["planes"].append(
                    frame_to_rhino_plane(
                        self.robot.from_BCF_to_WCF(frame_t)
                    )
                )
                self.results["positions"].append(c.positions)
                self.results["velocities"].append(c.velocities)
                self.results["accelerations"].append(c.accelerations)

            if self.validation:
                # Wait until trajectory is approved or replan is requested.
                while not stop_thread():
                    time.sleep(0.1)
                    if self.approved == True or self.replan == True:
                        break
                # Break if trajectory is approved.
                if self.approved == True:
                    break
                time.sleep(0.1)
            else:
                break
                
        self.is_completed = True
        return True

class InverseKinematicsTask(Task):
    def __init__(
        self,
        robot,
        frame_WCF,
        start_configuration,
        group="ur20",
        json_path=None,
        key=None,
    ):
        super(InverseKinematicsTask, self).__init__(key)
        self.robot = robot
        self.frame_WCF = frame_WCF
        self.start_configuration = start_configuration
        self.group = group
        self.configuration = None
        self.path = json_path

    def run(self, stop_thread):
        frame_BCF = self.frame_WCF.transformed(self.robot.transformation_WCF_BCF())

        self.log("Computing inverse kinematics...")
        self.configuration = self.robot.inverse_kinematics(
            frame_BCF, self.start_configuration, self.group
        )

        while not stop_thread():
            if self.configuration is not None:
                break
            time.sleep(0.1)

        self.log("Configuration found at {}.".format(self.configuration))
        filename = "Task_{}.json".format(self.key)
        filepath = self.path / filename
        json_data = json.dumps(self.configuration.to_data())

        with open(filepath, "w") as f:
            f.write(json_data)

        self.is_completed = True
        return True

class GetConfigurationTask(Task):
    def __init__(self, robot, key=None):
        super(GetConfigurationTask, self).__init__(key)
        self.robot = robot
        self.configuration = None

    def run(self, stop_thread):
        self.log("Waiting for current configuration...")
        current_joint_values = self.robot.mobile_client.current_joint_values

        joint_names_ordered = [
            "robot_ewellix_top_lift_joint",
            "robot_arm_shoulder_pan_joint",
            "robot_arm_shoulder_lift_joint",
            "robot_arm_elbow_joint",
            "robot_arm_wrist_1_joint",
            "robot_arm_wrist_2_joint",
            "robot_arm_wrist_3_joint",
        ]
        joint_values_ordered = [
            current_joint_values.get(joint_name, 0.00000)
            for joint_name in joint_names_ordered
        ]
        joint_types_ordered = [2, 0, 0, 0, 0, 0, 0]
        self.configuration = Configuration(joint_values_ordered, joint_types_ordered)

        self.log("Current configuration is: {}".format(self.configuration))

        self.is_completed = True
        return True

class ExecuteMotionTask(URTask):
    def __init__(self, robot, robot_address, fabrication, motiontask_key=0, reverse=False, velocity=0.06, radius=0.01, payload=0.0, CoG=[0,0,0], key=None):
        super(ExecuteMotionTask, self).__init__(robot, robot_address, key)
        self.robot = robot
        self.robot_address = robot_address
        self.fabrication = fabrication
        self.motiontask_key = motiontask_key
        self.velocity = velocity
        self.radius = radius
        self.configurations = None
        self.reverse = reverse
        self.payload = payload
        self.CoG = CoG
                
    def create_urscript(self):
        self.log("Executing the planned motion!")
        # Get the motion plan from 1 task before.
        motionplan_task = self.fabrication.get_task_by_key(self.motiontask_key)
        configurations = motionplan_task.results.get("configurations")
        frame_WCF = motionplan_task.frame_WCF
        if self.reverse:
            self.configurations = configurations[::-1]
        else:
            self.configurations = configurations

        self.urscript.set_payload(self.payload, self.CoG)
        self.urscript.add_line("textmsg(\">> TASK {}.\")".format(self.key))

        for config in self.configurations:
            joint_configuration = Configuration.from_revolute_values(config.revolute_values)
            self.urscript.move_joint(joint_configuration, self.velocity, self.radius)

        # Go to the target frame with radius 0.
        self.urscript.add_line("\tsleep({})".format(1.0))
        self.urscript.move_linear(frame=self.robot.from_WCF_to_RCF(frame_WCF), velocity=self.velocity/2, radius=0.0)
        #self.log(self.urscript.commands)

### Gripper related tasks ###

# Change attached tool frame of the robot model
class ChangeToolFrameTask(Task):
    def __init__(self, robot, tool_name='gripper', key=None):
        super(ChangeToolFrameTask, self).__init__(key)
        self.robot = robot
        self.tool_name = tool_name
        gripper_frame = Frame([0, 0, 0.166], [0, -1, 0], [1, 0, 0])
        camera_frame = Frame([-0.0327, 0.0572, 0.0815], [-1, 0, 0], [0, 0, 1])
        if tool_name == 'gripper':
            self.tool_frame = gripper_frame
        else:
            self.tool_frame = camera_frame
        
    def run(self, stop_thread):
        self.log("ChangeToolTask")
        self.log('Changing the tool frame to {}.'.format(self.tool_name))
        self.robot.attached_tools['ur20'].frame = self.tool_frame
        self.robot.attached_tools['ur20_and_liftkit'].frame = self.tool_frame
        self.is_completed = True
        return True

# IsaacSim gripper open/close
class SimGripperControlTask(Task):
    def __init__(self, robot, open_grip=True, key=None):
        super(SimGripperControlTask, self).__init__(key)
        self.robot = robot
        self.open_grip = open_grip
    
    def run(self, stop_thread):
        msg = {
            'data' : self.open_grip
        }

        time.sleep(1)
        while not stop_thread():
            if self.robot.mobile_client.is_service_available('/gripper_service'):
                result = self.robot.mobile_client.service_call('/gripper_service', 'std_srvs/srv/SetBool', msg)
                success = result['success']
                message = result['message']
                self.log('Message: {}'.format(message))
                if success:
                    break
            else:
                self.log("Gripper service is not available.")
                break
            time.sleep(0.1)
        time.sleep(1)
        self.is_completed = True
        return True

### UR direct tasks ###

class MoveJointsURdirectTask(URTask):
    def __init__(
        self,
        robot,
        robot_address,
        configuration,
        velocity=0.10,
        radius=0.0,
        payload=0.0,
        CoG=[0.0, 0.0, 0.0],
        key=None,
    ):
        super(MoveJointsURdirectTask, self).__init__(robot, robot_address, key)
        self.configuration = configuration
        self.velocity = velocity
        self.radius = radius
        self.payload = payload
        self.CoG = CoG

    def create_urscript(self):
        self.urscript.set_payload(self.payload, self.CoG)
        joint_configuration = Configuration.from_revolute_values(self.configuration.revolute_values)
        
        self.urscript.move_joint(joint_configuration, self.velocity, self.radius)
        self.log("Going to set configuration {}.".format(self.configuration))

class MoveLinearURdirectTask(URTask):
    def __init__(
        self,
        robot,
        robot_address,
        frame,
        in_RCF=True,
        velocity=0.10,
        radius=0.0,
        payload=0.0,
        CoG=[0.0, 0.0, 0.0],
        ee_transform=True,
        key=None,
    ):
        super(MoveLinearURdirectTask, self).__init__(robot, robot_address, key)
        self.robot = robot
        self.robot_address = robot_address
        self.frame = frame
        self.in_RCF = in_RCF
        self.velocity = velocity
        self.radius = radius
        self.payload = payload
        self.CoG = CoG
        self.ee_transform = ee_transform

    def create_urscript(self):
        if not self.in_RCF:
            frame_RCF = self.frame.transformed(self.robot.transformation_WCF_RCF())
        else:
            frame_RCF = self.frame
            
        # if self.ee_transform and self.robot.attached_tool:
        #     frame_RCF = self.robot.from_tcf_to_t0cf([frame_RCF])[0]
        #     self.log("Attached tool.")

        self.urscript.set_payload(self.payload, self.CoG)
        self.urscript.move_linear(frame_RCF, self.velocity, self.radius)

        self.log("Going to frame {}.".format(self.frame))

### UR direct force control action tasks ###

class DrillBrickURTask(URTask):
    def __init__(self, robot, robot_address, assembly, brick_key, top=True, middle=True, right=True, left=True, key=None):
        super(DrillBrickURTask, self).__init__(robot, robot_address, key)
        self.robot = robot
        self.robot_address = robot_address
        self.assembly = assembly
        self.brick_key = brick_key
        self.front_distance = 0.02
        self.top = top
        self.left = left
        self.middle = middle
        self.right = right

    def urscript_fabrication_header(self):
        ## Initialize instance
        self.urscript = URScript_Drill(*self.robot_address)
        self.urscript.start()
        
        if self.robot:
            ## Set tool
            tool = self.robot.attached_tool
            self.urscript.set_tcp(list(tool.frame.point)+list(tool.frame.axis_angle_vector))
        self.urscript.textmessage(">> TASK {}".format(self.key), string=True)
        
        if self.server:
            self.urscript.set_socket(self.server.ip, self.server.port, self.server.name)
            self.urscript.socket_open(self.server.name)
            ## Send script received msg
            self.urscript.socket_send_line_string(self.rec_msg, self.server.name)
    
    def stop_by_distance_and_force(self, urscript, max_distance, max_force, log_distance=False, log_force=False, log_max_force=True, indent=1):
        urscript.add_line("\tsleep({})".format(1.0), indent=indent)
        urscript.add_lines(["start_pose = get_actual_tcp_pose()"], indent=indent)

        urscript.add_lines(["last_force = 0", "last_distance = 0"], indent=indent)
        urscript.add_line("max_force_value = 0", indent=indent)
        
        urscript.add_line("while last_distance < {} and last_force < {}:".format(str(max_distance), str(abs(max_force))), indent=indent)
        urscript.add_line("sleep(0.01)", indent=indent+1)

        urscript.add_lines(["last_force = force()", "last_distance = norm(pose_sub(start_pose, get_actual_tcp_pose()))"], indent=indent+1)

        urscript.add_line("if last_force > max_force_value:", indent=indent+1)
        urscript.add_line("max_force_value = last_force", indent=indent+2)
        urscript.add_line("end", indent=indent+1)

        if log_distance:
            urscript.add_line("textmsg(last_distance)", indent=indent+1)

        if log_force:
            urscript.add_line("textmsg(last_force)", indent=indent+1)

        urscript.add_lines(["\tif last_force > {}:".format(str(abs(max_force))), "\t\tforce_end = True", "\telse:", "\t\tforce_end = False", "\tend"], indent=indent)
        urscript.add_line("end", indent=indent)

        urscript.add_lines(["if force_end == True:", '\ttextmsg("Forced to stop.")', "\tsleep({})".format(1.0), "\tend_force_mode()"], indent=indent)
        urscript.add_lines(["else:", "\tend_force_mode()", "\tsleep({})".format(2.0), "end"], indent=indent)

        urscript.add_line("\tsleep({})".format(1.0), indent=indent)

        urscript.add_line("last_distance = norm(pose_sub(start_pose, get_actual_tcp_pose()))", indent=indent)
        
        if log_max_force:
            urscript.add_line("textmsg(last_distance)", indent=indent)
            urscript.add_line("textmsg(max_force_value)", indent=indent)
    
    def adjust_vertical(self, urscript, max_distance, max_force=60, safety_force=90, original_frame=None, indent=1):
        urscript.add_lines(["while force_end == True:"], indent=indent)
        
        # Get back off.
        urscript.move_linear(original_frame, indent=indent+1)

        # Go down.
        urscript.move_tool_by_distance(x_distance=0.005, indent=indent+1)
        urscript.add_line("vertical_adjustment = vertical_adjustment + 1", indent=indent+1)

        # Try again.
        urscript.add_lines(["if vertical_adjustment < 2:"], indent=indent+1)
        urscript.move_force_mode(force_z=40, speed_z=0.015, indent=indent+2)
        self.stop_by_distance_and_force(urscript, max_distance=max_distance, max_force=max_force, log_distance=False, log_force=False, indent=indent+2)
        urscript.add_lines(["else:"], indent=indent+1)
        urscript.move_force_mode(force_z=40, speed_z=0.015, indent=indent+2)
        self.stop_by_distance_and_force(urscript, max_distance=max_distance, max_force=safety_force, log_distance=False, log_force=False, indent=indent+2)
        urscript.add_lines(["break"], indent=indent+2)
        urscript.add_lines(["end"], indent=indent+1)
        
        urscript.add_lines(["end"], indent=indent)

    def adjust_vertical_continue_horizontal(self, urscript, distance=0.13, direction=-1, safety_force=90, indent=1):
        urscript.add_lines(["if force_end == True:"], indent=indent)
        urscript.add_line("sleep(0.01)", indent=indent+1)

        urscript.add_lines(["if vertical_adjustment < 2:"], indent=indent+1)
        urscript.move_tool_by_distance(x_distance=0.005, indent=indent+2)
        urscript.add_line("vertical_adjustment = vertical_adjustment + 1", indent=indent+2)
        urscript.add_line('textmsg("Adjusted drill vertically.")', indent=indent+2)
        urscript.add_line("textmsg(vertical_adjustment)", indent=indent+2)
        urscript.add_lines(["else:"], indent=indent+1)
        urscript.add_line('textmsg("Cannot adjust vertically anymore.")', indent=indent+2)
        urscript.add_lines(["end"], indent=indent+1)

        abs_distance = abs(distance)

        urscript.add_line("remaining_distance = {} - last_distance".format(abs_distance), indent=indent+1)
        urscript.add_line('textmsg("Remanining distance is...")', indent=indent+1)
        urscript.add_line("textmsg(remaining_distance)", indent=indent+1)

        self.move_force_mode(urscript, force_y=80 * direction, speed_y=0.015, indent=indent+1)
        self.stop_by_distance_and_force(urscript, max_distance="remaining_distance", max_force=safety_force, log_distance=False, log_force=False, indent=indent+1)

        urscript.add_lines(["end"], indent=indent)

    def continue_vertical(self, urscript, distance=0.08, direction=-1, safety_force=90, indent=1):
        urscript.add_lines(["if force_end == True:"], indent=indent)
        urscript.add_line("sleep(0.01)", indent=indent+1)
        
        abs_distance = abs(distance)

        urscript.add_line("remaining_distance = {} - last_distance".format(abs_distance), indent=indent+1)
        urscript.add_line('textmsg("Remanining distance is...")', indent=indent+1)
        urscript.add_line("textmsg(remaining_distance)", indent=indent+1)

        self.move_force_mode(urscript, force_x=80 * direction, speed_x=0.015, indent=indent+1)
        self.stop_by_distance_and_force(urscript, max_distance="remaining_distance", max_force=safety_force, log_distance=False, log_force=False, indent=indent+1)

        urscript.add_lines(["end"], indent=indent)

    def move_force_mode(self, urscript, force_x=0.0, speed_x=0.01, force_y=0.0, speed_y=0.01, force_z=0.0, speed_z=0.01, indent=1):
        """Get the robot in the force mode.

        Parameters
        ----------
        max force : float
            Force limit in N (Newton) for the z axis.
            10.0
        max speed : float
            Speed limit in m/s for the z axis.
            0.025

        Returns
        -------
        None
            Robot is in the force mode in defined axes.
        """
        if force_x != 0.0:
            x = 1
        else:
            x = 0
        if force_y != 0.0:
            y = 1
        else:
            y = 0
        if force_z != 0.0:
            z = 1
        else:
            z = 0
        urscript.force_mode([x, y, z, 0, 0, 0], [force_x, force_y, force_z, 0.0, 0.0, 0.0], [speed_x, speed_y, speed_z, 0.03, 0.03, 0.03], indent=indent)

    def drill_top_section(self, urscript, left_top_drill_frame):
        cleaning_inner_distances = [0.02, 0.03, 0.03]

        urscript.move_linear(left_top_drill_frame)
        urscript.drill_on()

        for i, cleaning_inner_distance in enumerate(cleaning_inner_distances):

            if i == 0:
                cleaning_inner_distance = cleaning_inner_distance + self.front_distance

            # Go in 3 cm.
            urscript.add_line("\tsleep({})".format(1.0))
            urscript.move_force_mode(force_z=40, speed_z=0.015)
            self.stop_by_distance_and_force(urscript, max_distance=cleaning_inner_distance, max_force=100, log_distance=False, log_force=False)

            if i % 2 == 0:
                # Go right, direction -1.
                direction = -1
            else:
                # Go left, direction +1.
                direction = 1

            # Go horizontal.
            self.move_force_mode(urscript, force_y=80*direction, speed_y=0.015)
            self.stop_by_distance_and_force(urscript, max_distance=0.26, max_force=100, log_distance=False, log_force=False)

        # Go out front distance + all way in.
        urscript.move_force_mode(force_z=-40, speed_z=0.015)
        self.stop_by_distance_and_force(urscript, max_distance=(self.front_distance+sum(cleaning_inner_distances)), max_force=150, log_distance=False, log_force=False)

        urscript.drill_off()
        
    def drill_middle_bottom_section(self, urscript, bottom_drill_frame):
        length_middle_section = 0.12
        middle_inner_distances = [0.026, 0.063, 0.1] #[0.023, 0.06, 0.097]

        middle_left_bottom_drill_frame = bottom_drill_frame.transformed(Translation.from_vector(bottom_drill_frame.yaxis * (length_middle_section/2)))

        for middle_inner_distance in middle_inner_distances:
            urscript.move_linear(middle_left_bottom_drill_frame)

            urscript.move_tool_by_distance(x_distance="0.005 * vertical_adjustment")

            urscript.drill_on()

            max_force = 75 #80
            safety_force = 100 #100

            urscript.move_force_mode(force_z=40, speed_z=0.015)
            self.stop_by_distance_and_force(urscript, max_distance=(self.front_distance+middle_inner_distance), max_force=80, log_distance=False, log_force=False)
            self.adjust_vertical(urscript, max_distance=(self.front_distance+middle_inner_distance), max_force=80, safety_force=safety_force, original_frame=middle_left_bottom_drill_frame)

            # Go horizontal right.
            self.move_force_mode(urscript, force_y=-80, speed_y=0.015)
            tolerance = 0.015
            self.stop_by_distance_and_force(urscript, max_distance=length_middle_section-tolerance, max_force=max_force, log_distance=False, log_force=False)
            self.adjust_vertical_continue_horizontal(urscript, distance=length_middle_section-tolerance, direction=-1, safety_force=safety_force)

            urscript.move_force_mode(force_z=-40, speed_z=0.015)
            self.stop_by_distance_and_force(urscript, max_distance=(self.front_distance+middle_inner_distance), max_force=80, log_distance=False, log_force=False)

            urscript.drill_off()
            
    def drill_right_bottom_section(self, urscript, bottom_drill_frame):
        length_right_section = 0.12 #0.1
        length_middle_section = 0.12
        break_width = 0.025 #0.03
        inner_distances = [0.026, 0.063, 0.1]

        middle_right_bottom_drill_frame = bottom_drill_frame.transformed(Translation.from_vector(-bottom_drill_frame.yaxis * (length_middle_section/2+break_width)))

        max_force = 75 #80
        safety_force = 100 #100

        for inner_distance in inner_distances:
            urscript.move_linear(middle_right_bottom_drill_frame)

            urscript.move_tool_by_distance(x_distance="0.005 * vertical_adjustment")

            urscript.drill_on()

            urscript.move_force_mode(force_z=40, speed_z=0.015)
            self.stop_by_distance_and_force(urscript, max_distance=(self.front_distance+inner_distance), max_force=80, log_distance=False, log_force=False)
            self.adjust_vertical(urscript, max_distance=(self.front_distance+inner_distance), max_force=80, safety_force=safety_force, original_frame=middle_right_bottom_drill_frame)

            # Go horizontal right.
            self.move_force_mode(urscript, force_y=-80, speed_y=0.015)
            tolerance = 0.015
            self.stop_by_distance_and_force(urscript, max_distance=length_right_section-tolerance, max_force=max_force, log_distance=False, log_force=False)
            self.adjust_vertical_continue_horizontal(urscript, distance=length_right_section-tolerance, direction=-1, safety_force=safety_force)

            urscript.move_force_mode(force_z=-40, speed_z=0.015)
            self.stop_by_distance_and_force(urscript, max_distance=(self.front_distance+inner_distance-tolerance), max_force=80, log_distance=False, log_force=False)

            urscript.drill_off()

    def drill_left_bottom_section(self, urscript, bottom_drill_frame):
        length_left_section = 0.055 #0.045 #0.06
        length_middle_section = 0.12
        break_width = 0.025
        inner_distances = [0.026, 0.063, 0.1]

        max_force = 80 #80
        safety_force = 100 #100

        middle_left_bottom_drill_frame = bottom_drill_frame.transformed(Translation.from_vector(bottom_drill_frame.yaxis * (length_middle_section/2+break_width)))

        for inner_distance in inner_distances:
            urscript.move_linear(middle_left_bottom_drill_frame)

            urscript.move_tool_by_distance(x_distance="0.005 * vertical_adjustment")

            urscript.drill_on()

            urscript.move_force_mode(force_z=40, speed_z=0.015)
            self.stop_by_distance_and_force(urscript, max_distance=(self.front_distance+inner_distance), max_force=80, log_distance=False, log_force=False)
            self.adjust_vertical(urscript, max_distance=(self.front_distance+inner_distance), max_force=80, safety_force=safety_force, original_frame=middle_left_bottom_drill_frame)

            # Go horizontal left.
            self.move_force_mode(urscript, force_y=80, speed_y=0.015)
            tolerance = 0.015
            self.stop_by_distance_and_force(urscript, max_distance=length_left_section-tolerance, max_force=max_force, log_distance=False, log_force=False)
            self.adjust_vertical_continue_horizontal(urscript, distance=length_left_section-tolerance, direction=1, safety_force=safety_force)

            # Go up.
            self.move_force_mode(urscript, force_x=-80, speed_x=0.015)
            self.stop_by_distance_and_force(urscript, max_distance=0.08, max_force=80, log_distance=False, log_force=False)
            self.continue_vertical(urscript, distance=0.08, direction=-1, safety_force=safety_force)

            # Go out.
            urscript.move_force_mode(force_z=-40, speed_z=0.015)
            self.stop_by_distance_and_force(urscript, max_distance=(self.front_distance+inner_distance), max_force=80, log_distance=False, log_force=False)

            urscript.drill_off()
                
    def create_urscript(self):
        
        brick_frame = self.assembly.part(self.brick_key).frame
        front_frame = brick_frame.transformed(Translation.from_vector(brick_frame.xaxis * (self.front_distance + 0.055)))

        drill_frame = front_frame.transformed(Rotation.from_axis_and_angle(front_frame.yaxis, math.radians(-90), front_frame.point))
        top_drill_frame = drill_frame.transformed(Translation.from_vector(-drill_frame.xaxis * (0.025 + 0.015))) #(0.025 + 0.015)
        left_top_drill_frame = top_drill_frame.transformed(Translation.from_vector(top_drill_frame.yaxis * 0.13))
        bottom_drill_frame = drill_frame.transformed(Translation.from_vector(drill_frame.xaxis * (0.025 + 0.015))) #(0.025 + 0.015)
        left_bottom_drill_frame = bottom_drill_frame.transformed(Translation.from_vector(bottom_drill_frame.yaxis * 0.15))

        self.urscript.set_payload(5.6, [0.005, -0.022, 0.072])
        
        self.log("Drilling started!")
        # For adjustment to only once.
        self.urscript.add_line("vertical_adjustment = 0")

        # Clean top, middle, right and left in order.
        if self.top:
            self.drill_top_section(self.urscript, left_top_drill_frame)
        if self.middle:
            self.drill_middle_bottom_section(self.urscript, bottom_drill_frame)
        if self.right:
            self.drill_right_bottom_section(self.urscript, bottom_drill_frame)
        if self.left:
            self.drill_left_bottom_section(self.urscript, bottom_drill_frame)
        high_drill_frame = top_drill_frame.transformed(Translation.from_vector(-top_drill_frame.xaxis * (0.15)))
        high_in_drill_frame = high_drill_frame.transformed(Translation.from_vector(high_drill_frame.zaxis * (0.2)))
        self.urscript.move_linear(high_drill_frame)
        self.urscript.move_linear(high_in_drill_frame)

class PickBrickURTask(URTask):
    def __init__(self, robot, robot_address, assembly, brick_key, grip=True, key=None):
        super(PickBrickURTask, self).__init__(robot, robot_address, key)
        self.robot = robot
        self.robot_address = robot_address
        self.assembly = assembly
        self.brick_key = brick_key
        self.grip = grip

    def urscript_fabrication_header(self):
        ## Initialize instance
        self.urscript = URScript_ParallelGrip(*self.robot_address)
        self.urscript.start()
        
        if self.robot:
            ## Set tool
            tool = self.robot.attached_tool
            self.urscript.set_tcp(list(tool.frame.point)+list(tool.frame.axis_angle_vector))
        self.urscript.textmessage(">> TASK {}".format(self.key), string=True)
        
        if self.server:
            self.urscript.set_socket(self.server.ip, self.server.port, self.server.name)
            self.urscript.socket_open(self.server.name)
            ## Send script received msg
            self.urscript.socket_send_line_string(self.rec_msg, self.server.name)
    
    def create_urscript(self):
        brick_frame = self.assembly.find_by_key(self.brick_key).frame.transformed(Translation.from_vector(Vector.Zaxis()*0.025))
        brick_pose = brick_frame.point.__data__ + brick_frame.axis_angle_vector.__data__
        
        self.log("Picking started!")
        self.urscript.set_payload(5.6, [0.005, -0.022, 0.072])
        self.urscript.parallelgrip_open()

        # Move down 1st try.
        self.urscript.move_force_mode(force_z=20.0, speed_z=0.015)
        self.urscript.stop_by_force(15.0)

        # Check the x force.
        self.urscript.add_line("x_force = get_tcp_force()[0]")
        # urscript.add_lines(["textmsg(x_force)"])
        self.urscript.add_line("brick_pose = p[{}, {}, {}, {}, {}, {}]".format(*brick_pose))
        self.urscript.add_line("distance_taken = 0.001")

        # Check if in close 3 cm (in z axis), if not move around and try again.
        self.urscript.add_line("while norm(brick_pose[2] - get_actual_tcp_pose()[2]) > 0.037 or distance_taken > 0.01:", indent=1)
        self.urscript.add_line("textmsg(norm(brick_pose[2] - get_actual_tcp_pose()[2]))", indent=2)

        self.urscript.add_line("pose_new = get_actual_tcp_pose()", indent=2)
        self.urscript.move_tool_by_distance(z_distance=-0.002, indent=2)
        self.urscript.add_line("sleep({})".format(1.0), indent=2)

        self.urscript.add_line("if x_force > 0:", indent=2)
        self.urscript.move_tool_by_distance(x_distance=-0.008, indent=3)
        self.urscript.add_line("else:", indent=2)
        self.urscript.move_tool_by_distance(x_distance=0.008, indent=3)
        self.urscript.add_line("end", indent=2)

        # Move down again.
        self.urscript.add_line("sleep({})".format(1.0), indent=2)
        self.urscript.move_force_mode(force_z=20.0, speed_z=0.015, indent=2)
        self.urscript.stop_by_force(15.0,indent=2)
        self.urscript.add_line("x_force = get_tcp_force()[0]",indent=2)
        # urscript.add_lines(["textmsg(x_force)"], indent=2)

        self.urscript.add_line("pose_new_low = get_actual_tcp_pose()", indent=2)
        self.urscript.add_line("distance_taken = pose_dist(pose_new, pose_new_low)", indent=2)

        self.urscript.add_line("end", indent=1)

        self.urscript.textmessage("Got the brick.", string=True)

        # Find middle:
        # Move up a little.
        self.urscript.move_tool_by_distance(z_distance=-0.003)
        self.urscript.add_line("sleep({})".format(2.0))

        # Go in x to get a mid point.
        self.urscript.move_force_mode(force_x=-10.0, speed_x=0.010)
        self.urscript.stop_by_force(15.0)

        # Go to middle.
        self.urscript.add_line("sleep({})".format(1.0))
        self.urscript.move_tool_by_distance(x_distance=0.0075)
        self.urscript.add_line("sleep({})".format(1.0))

        # Move down.
        self.urscript.move_force_mode(force_z=15.0, speed_z=0.010)
        self.urscript.stop_by_force(15.0)

        if self.grip:
            self.urscript.parallelgrip_close()

class BreakBrickURTask(URTask):
    def __init__(self, robot, robot_address, assembly, brick_key=0, reverse=False, key=None):
        super(BreakBrickURTask, self).__init__(robot, robot_address, key)
        self.robot = robot
        self.robot_address = robot_address
        self.assembly = assembly
        self.brick_key = brick_key
        self.reverse = reverse

    def urscript_fabrication_header(self):
        
        ## Initialize instance
        self.urscript = URScript_ParallelGrip(*self.robot_address)
        self.urscript.start()
        
        if self.robot:
            ## Set tool
            tool = self.robot.attached_tool
            self.urscript.set_tcp(list(tool.frame.point)+list(tool.frame.axis_angle_vector))
        self.urscript.textmessage(">> TASK {}".format(self.key), string=True)
        
        if self.server:
            self.urscript.set_socket(self.server.ip, self.server.port, self.server.name)
            self.urscript.socket_open(self.server.name)
            ## Send script received msg
            self.urscript.socket_send_line_string(self.rec_msg, self.server.name)
                
    def create_urscript(self):
        self.log("Breaking started!")
        self.urscript.set_payload(8.6, [0.005, -0.022, 0.072])
        original_frame = self.assembly.part(self.brick_key).frame
        if self.reverse:
            rev = -1
        else:
            rev = 1
        brick_frame = Frame(original_frame.point, rev*Vector(original_frame.xaxis.x, original_frame.xaxis.y), rev*original_frame.yaxis)

        pick_frame = brick_frame.transformed(Translation.from_vector(-brick_frame.zaxis * 0.08))
        rotated_frame = pick_frame.transformed(Rotation.from_axis_and_angle(pick_frame.yaxis, 
                                                                            math.radians(-20), 
                                                                            pick_frame.transformed(Translation.from_vector(pick_frame.xaxis * 0.055)).point))
        rotated_frame_safe = rotated_frame.transformed(Translation.from_vector(-rotated_frame.zaxis * 0.10))
        pick_frame_safe = brick_frame.transformed(Translation.from_vector(-brick_frame.zaxis * 0.20))

        self.urscript.move_linear(rotated_frame, velocity=0.05, radius=0.0)
                
        # Move to safe frame.
        self.urscript.move_linear(rotated_frame_safe, velocity=0.05, radius=0.00)
        self.urscript.move_linear(pick_frame_safe, velocity=0.05, radius=0.00)

class PlaceBrickURTask(URTask):
    def __init__(self, robot, robot_address, release=True, key=None):
        super(PlaceBrickURTask, self).__init__(robot, robot_address, key)
        self.robot = robot
        self.robot_address = robot_address
        self.release = release

    def urscript_fabrication_header(self):
        ## Initialize instance
        self.urscript = URScript_ParallelGrip(*self.robot_address)
        self.urscript.start()
        
        if self.robot:
            ## Set tool
            tool = self.robot.attached_tool
            self.urscript.set_tcp(list(tool.frame.point)+list(tool.frame.axis_angle_vector))
        self.urscript.textmessage(">> TASK {}".format(self.key), string=True)
        
        if self.server:
            self.urscript.set_socket(self.server.ip, self.server.port, self.server.name)
            self.urscript.socket_open(self.server.name)
            ## Send script received msg
            self.urscript.socket_send_line_string(self.rec_msg, self.server.name)
                
    def create_urscript(self):
        self.log("Placing started!")
        self.urscript.set_payload(8.6, [0.005, -0.022, 0.072])

        self.urscript.parallelgrip_close()

        self.urscript.add_line("\tsleep({})".format(1.0))

        self.urscript.move_force_mode(force_z=50.0, speed_z=0.03)
        self.urscript.stop_by_force(20.0)

        if self.release:
            self.urscript.parallelgrip_open()
        
        self.urscript.set_payload(5.6, [0.005, -0.022, 0.072])
        self.urscript.move_tool_by_distance(z_distance=-0.8, velocity=0.1, radius=0.01)
        self.urscript.parallelgrip_close()


class PlaceBrickURTask(URTask):
    def __init__(self, robot, robot_address, release=True, key=None):
        super(PlaceBrickURTask, self).__init__(robot, robot_address, key)
        self.robot = robot
        self.robot_address = robot_address
        self.release = release

    def urscript_fabrication_header(self):
        ## Initialize instance
        self.urscript = URScript_ParallelGrip(*self.robot_address)
        self.urscript.start()
        
        if self.robot:
            ## Set tool
            tool = self.robot.attached_tool
            self.urscript.set_tcp(list(tool.frame.point)+list(tool.frame.axis_angle_vector))
        self.urscript.textmessage(">> TASK {}".format(self.key), string=True)
        
        if self.server:
            self.urscript.set_socket(self.server.ip, self.server.port, self.server.name)
            self.urscript.socket_open(self.server.name)
            ## Send script received msg
            self.urscript.socket_send_line_string(self.rec_msg, self.server.name)
                
    def create_urscript(self):
        self.log("Placing started!")
        self.urscript.set_payload(8.6, [0.005, -0.022, 0.072])

        self.urscript.parallelgrip_close()

        self.urscript.add_line("\tsleep({})".format(1.0))

        self.urscript.move_force_mode(force_z=50.0, speed_z=0.03)
        self.urscript.stop_by_force(20.0)

        if self.release:
            self.urscript.parallelgrip_open()
        
        self.urscript.set_payload(5.6, [0.005, -0.022, 0.072])
        self.urscript.move_tool_by_distance(z_distance=-0.8, velocity=0.1, radius=0.01)
        self.urscript.parallelgrip_close()
### Marker related tasks ###

class SearchAndSaveMarkersTask(Task):
    def __init__(
        self, robot, robot_address, fabrication, duration=10, update=True, key=None
    ):
        super(SearchAndSaveMarkersTask, self).__init__(key)
        self.robot = robot
        self.robot_address = robot_address
        self.fabrication = fabrication
        self.duration = duration
        self.update = update
        self.marker_ids = []

    def receive_marker_ids(self, message):
        msg = message.get("transforms")[0]
        if msg.get("header").get("frame_id") == "camera_color_optical_frame":
            marker_id = msg.get("child_frame_id")
            if marker_id not in self.marker_ids:
                self.log("Found marker with ID: {}".format(marker_id))
                if (self.update) or (
                    not self.update
                    and not self.robot.mobile_client.marker_frames.get(marker_id)
                ):
                    self.marker_ids.append(marker_id)
                else:
                    self.log(
                        "Ignoring {}, as it is already recorded in the marker dictionary and update is set to False.".format(
                            marker_id
                        )
                    )

    def run(self, stop_thread):
        self.marker_ids = []
        # Get the marker ids in the scene
        self.robot.mobile_client.topic_subscribe(
            "/tf", "tf2_msgs/TFMessage", self.receive_marker_ids
        )
        t0 = time.time()
        while time.time() - t0 < self.duration and not stop_thread():
            time.sleep(0.1)
        self.robot.mobile_client.topic_unsubscribe("/tf")
        self.log("Got all the visible marker ids.")
        time.sleep(1)
        self.log("Length of the list is {}.".format(len(self.marker_ids)))

        # Iterate the marker ids.
        if len(self.marker_ids) > 0:
            for marker_id in self.marker_ids:
                next_key = self.fabrication.get_next_task_key()
                task = GetMarkerPoseTask(
                    self.robot,
                    marker_id=marker_id,
                    reference_frame_id="robot_arm_base",
                    key=next_key,
                )
                self.fabrication.add_task(task, key=next_key)
        else:
            self.log("No more markers are visible.")

        self.is_completed = True
        return True

class SearchAndSaveRobotPoseInMarkerTask(Task):
    def __init__(
        self, robot, robot_address, fabrication, duration=10, update=True, key=None
    ):
        super(SearchAndSaveRobotPoseInMarkerTask, self).__init__(key)
        self.robot = robot
        self.robot_address = robot_address
        self.fabrication = fabrication
        self.duration = duration
        self.update = update
        self.marker_ids = []

    def receive_marker_ids(self, message):
        msg = message.get("transforms")[0]
        if msg.get("header").get("frame_id") == "camera_color_optical_frame":
            marker_id = msg.get("child_frame_id")
            if marker_id not in self.marker_ids:
                self.log("Found marker with ID: {}".format(marker_id))
                if (self.update) or (
                    not self.update
                    and not self.robot.mobile_client.marker_frames.get(marker_id)
                ):
                    self.marker_ids.append(marker_id)
                else:
                    self.log(
                        "Ignoring {}, as it is already recorded in the marker dictionary and update is set to False.".format(
                            marker_id
                        )
                    )

    def run(self, stop_thread):
        self.marker_ids = []
        # Get the marker ids in the scene
        self.robot.mobile_client.topic_subscribe(
            "/tf", "tf2_msgs/TFMessage", self.receive_marker_ids
        )
        t0 = time.time()
        while time.time() - t0 < self.duration and not stop_thread():
            time.sleep(0.1)
        self.robot.mobile_client.topic_unsubscribe("/tf")
        self.log("Got all the visible marker ids.")
        time.sleep(1)
        self.log("Length of the list is {}.".format(len(self.marker_ids)))

        # Iterate the marker ids.
        if len(self.marker_ids) > 0:
            for marker_id in self.marker_ids:
                next_key = self.fabrication.get_next_task_key()
                task = GetRobotPoseInMarkerPoseTask(
                    self.robot,
                    marker_id=marker_id,
                    reference_frame_id="robot_arm_base",
                    key=next_key,
                )
                self.fabrication.add_task(task, key=next_key)
        else:
            self.log("No more markers are visible.")

        self.is_completed = True
        return True

class GetRobotPoseInMarkerPoseTask(Task):
    def __init__(
        self, robot, marker_id="marker_0", reference_frame_id="robot_arm_base", key=None
    ):
        super(GetRobotPoseInMarkerPoseTask, self).__init__(key)
        self.robot = robot
        self.marker_id = marker_id
        self.reference_frame_id = reference_frame_id

    def run(self, stop_thread):
        self.robot.mobile_client.clean_tf_frame()
        self.robot.mobile_client.tf_subscribe(self.marker_id, self.reference_frame_id)
        t0 = time.time()
        while (
            time.time() - t0 < 20 and not stop_thread()
        ):  # can be used for live subscription when time limit is removed.
            time.sleep(0.1)
            if self.robot.mobile_client.tf_frame is not None:
                MCF_in_RCF = Frame(
                    self.robot.mobile_client.tf_frame.point,
                    self.robot.mobile_client.tf_frame.zaxis,
                    -self.robot.mobile_client.tf_frame.yaxis,
                )
                MCF_in_BCF = MCF_in_RCF.transformed(self.robot.transformation_RCF_BCF())
                BCF_in_MCF = Frame.from_transformation(
                    Transformation.from_frame(MCF_in_BCF).inverted()
                )  # Invert
                self.log(
                    "Robot base frame in reference to {} is {}.".format(
                        self.marker_id, BCF_in_MCF
                    )
                )

                # Marker frames are added to the dict in WCF.
                self.robot.mobile_client.marker_frames[self.marker_id] = BCF_in_MCF
                break
        if self.robot.mobile_client.tf_frame is None:
            self.log("For {}, could not get the frame.".format(self.marker_id))
        self.robot.mobile_client.tf_unsubscribe(self.marker_id, self.reference_frame_id)
        self.is_completed = True
        return True

class GetMarkerPoseTask(Task):
    def __init__(
        self, robot, marker_id="marker_0", reference_frame_id="robot_arm_base", key=None
    ):
        super(GetMarkerPoseTask, self).__init__(key)
        self.robot = robot
        self.marker_id = marker_id
        self.reference_frame_id = reference_frame_id

    def run(self, stop_thread):
        self.robot.mobile_client.clean_tf_frame()
        self.robot.mobile_client.tf_subscribe(self.marker_id, self.reference_frame_id)
        t0 = time.time()
        while (
            time.time() - t0 < 20 and not stop_thread()
        ):  # can be used for live subscription when time limit is removed.
            time.sleep(0.1)

            if self.robot.mobile_client.tf_frame is not None:
                MCF_in_RCF = Frame(
                    self.robot.mobile_client.tf_frame.point,
                    self.robot.mobile_client.tf_frame.zaxis,
                    -self.robot.mobile_client.tf_frame.yaxis,
                )
                MCF_in_BCF = MCF_in_RCF.transformed(self.robot.transformation_RCF_BCF())
                self.log(
                    "{} pose in reference to robot base frame is {}.".format(
                        self.marker_id, MCF_in_BCF
                    )
                )
                # Marker frames are added to the dict in WCF.
                self.robot.mobile_client.marker_frames[self.marker_id] = MCF_in_BCF
                break
        if self.robot.mobile_client.tf_frame is None:
            self.log("For {}, could not get the frame.".format(self.marker_id))
        self.robot.mobile_client.tf_unsubscribe(self.marker_id, self.reference_frame_id)
        self.is_completed = True
        return True

class FixRobotToMarkerTask(Task):
    def __init__(self, robot, fixed_marker_id="marker_0", key=None):
        super(FixRobotToMarkerTask, self).__init__(key)
        self.robot = robot
        self.fixed_marker_id = fixed_marker_id
        self.marker_pose = None

    def run(self, stop_thread):
        # Get the frame of the fixed marker id.
        self.robot.mobile_client.clean_tf_frame()
        self.robot.mobile_client.tf_subscribe(self.fixed_marker_id, "robot_arm_base")
        t0 = time.time()
        while (
            time.time() - t0 < 20 and not stop_thread()
        ):  # can be used for live subscription when time limit is removed.
            time.sleep(0.1)
            if self.robot.mobile_client.tf_frame is not None:
                self.log(
                    "For {}, got the frame: {}".format(
                        self.fixed_marker_id, self.robot.mobile_client.tf_frame
                    )
                )
                self.marker_pose = self.robot.mobile_client.tf_frame
                break
        if self.robot.mobile_client.tf_frame is None:
            self.log("For {}, could not get the frame.".format(self.fixed_marker_id))
        self.robot.mobile_client.tf_unsubscribe(self.fixed_marker_id, "robot_arm_base")

        # Fix the robot to the marker pose
        if self.marker_pose is not None:
            MCF_in_RCF = self.marker_pose  # marker frame in RCF
            MCF_in_BCF = MCF_in_RCF.transformed(
                self.robot.transformation_RCF_BCF()
            )  # marker frame in BCF
            BCF_in_MCF = Frame.from_transformation(
                Transformation.from_frame(MCF_in_BCF).inverted()
            )  # BCF in measured MCF
            MCF_in_WCF = self.robot.mobile_client.marker_frames[
                self.fixed_marker_id
            ]  # marker frame in WCF
            from_MCF_to_WCF = Transformation.from_change_of_basis(
                MCF_in_WCF, Frame.worldXY()
            )  # T from fixed MCF to WCF

            BCF_in_WCF = BCF_in_MCF.transformed(from_MCF_to_WCF)  # BCF in WCF

            self.robot.BCF = BCF_in_WCF
            self.log("Robot is fixed to {}.".format(self.fixed_marker_id))
        else:
            self.log("Fixed marker frame is not retrieved.")

        self.is_completed = True
        return True

if __name__ == "__main__":
    pass
