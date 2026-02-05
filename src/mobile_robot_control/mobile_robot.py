from compas_fab.robots import Robot

from compas.geometry import Frame, Point, Vector
from compas.geometry import Transformation, Translation, Quaternion
from roslibpy import Message, Topic, Service, tf
import time
import json

__all__ = ["MobileRobot"]


class MobileRobot(Robot):
    """Represents a robot, which can be moved in the world coordinate system."""

    def __init__(
        self,
        model,
        artist=None,
        semantics=None,
        client=None,
        mobile_client=None,
        **kwargs,
    ):
        super(MobileRobot, self).__init__(model, artist, semantics, client)

        """
        documentation
        """
        self._scale_factor = 1.0
        self.model = model
        self.artist = artist
        self.semantics = semantics
        self.client = client
        self.mobile_client = mobile_client
        self.tf_terminology = {"RCF" : "robot_arm_base",
                               "BCF" : "robot_base_footprint"}
        
        self.attributes = {}
        self._current_ik = {"request_id": None, "solutions": None}
        
        self.motion_enabled = False # whether the mobile robot is moving
        self.is_aligned = False # whether the WCF_slam (okvis world) is aligned with WCF (rhino world)

        self._lift_height = 0  # lift height

        self._WCF = Frame.worldXY()  # world coordinate frame (WCF)
        self._BCF = Frame.worldXY()  # base coordinate frame in WCF (BCF)
        self._RCF = None  # ur robot arm coordinate frame in BCF (RCF)

        self._WCF_slam = Frame.worldXY()  # reference (okvis) world coordinate frame in WCF (WCF_slam) 
        self._BCF_slam = Frame.worldXY()  # (okvis) base coordinate frame in WCF_slam (BCF_slam)
        
        self._BCF_gt = Frame.worldXY()  # ground truth IsaacSim base coordinate frame in WCF (BCF_gt) 
        
        self._PCF = Frame.worldXY()  # fixed element pick frame on mobile robot's base in RCF (PCF)
        
        self._frame_log = []
        
        self._update_RCF()
        self._update_BCF() # compute initial BCF
        self._record_state("init") # record initial state
        
    @property
    def WCF(self):
        """Rhino World Coordinate Frame (always worldXY)."""
        return self._WCF
    
    @property
    def WCF_slam(self):
        """Reference World Coordinate Frame. Transformation between OKVIS and Rhino WCFs."""
        return self._WCF_slam

    @WCF_slam.setter
    def WCF_slam(self, frame_or_transform):
        # You can either give a Frame (for convenience) or a Transformation
        if isinstance(frame_or_transform, Frame):
            self._WCF_slam = frame_or_transform
        elif isinstance(frame_or_transform, Transformation):
            self._WCF_slam = Frame.worldXY().transformed(frame_or_transform)
        else:
            raise TypeError("WCF_slam must be a Frame or Transformation")
        self._update_BCF()
        
    @property
    def BCF_slam(self):
        """Base Coordinate Frame in OKVIS WCF_slam."""
        return self._BCF_slam

    @BCF_slam.setter
    def BCF_slam(self, frame):
        if not isinstance(frame, Frame):
            raise TypeError("BCF_slam must be a compas Frame")
        self._BCF_slam = frame
        self._update_BCF()
    
    @property
    def BCF(self):
        """Robot base frame in Rhino world = WCF_slam x BCF_slam"""
        return self._BCF
    
    def _update_BCF(self):
        """BCF = WCF_slam x BCF_slam"""
        T_rwcf = Transformation.from_frame(self._WCF_slam)
        self._BCF = self._BCF_slam.transformed(T_rwcf)
        
    @property
    def BCF_gt(self):
        return self._BCF_gt

    @BCF_gt.setter
    def BCF_gt(self, frame):
        if not isinstance(frame, Frame):
            raise TypeError("BCF_gt must be a compas.geometry.Frame")
        self._BCF_gt = frame
        
    @property
    def RCF(self):
        return self._RCF
    
    def _update_RCF(self, RCF_frame=None):
        if RCF_frame is not None:
            self._RCF = RCF_frame
        elif self.mobile_client != None:
            self.mobile_client.tf_subscribe(
                self.tf_terminology["RCF"],
                self.tf_terminology["BCF"],
                self._receive_base_frame_callback,
                timeout=5,
            )
        else:
            self._RCF = Frame(Point(0.275, 0.0, 1.0328), Vector(-0.707, 0.707, 0.0), Vector(-0.707, -0.707, 0.0))
        
        #     robot_arm_base_link = self.forward_kinematics(self.zero_configuration(), 'ur10e', True, options={'link':'robot_arm_base_link'})
        #     self._RCF = Frame(robot_arm_base_link.point, -robot_arm_base_link.xaxis, -robot_arm_base_link.yaxis)
        # if self.wheel_type == 'outdoor':
        #     self._RCF = Frame(Point(0.275, 0.0, 1.049 + self.lift_height), Vector(-0.707, 0.707, 0.0), Vector(-0.707, -0.707, 0.0))
        # elif self.wheel_type == 'indoor':
        # self._RCF = Frame(Point(0.275, 0.0, 1.021 + self.lift_height), Vector(-0.707, 0.707, 0.0), Vector(-0.707, -0.707, 0.0))
        
        return self._RCF

    def _receive_base_frame_callback(self, message):
        pose_point = Point(
            message["translation"]["x"],
            message["translation"]["y"],
            message["translation"]["z"],
        )
        pose_quaternion = Quaternion(
            message["rotation"]["w"],
            message["rotation"]["x"],
            message["rotation"]["y"],
            message["rotation"]["z"],
        )
        pose_frame = Frame.from_quaternion(pose_quaternion, pose_point)
        self._RCF = pose_frame

        return self._RCF

    @property
    def lift_height(self):
        return self._lift_height

    @lift_height.setter
    def lift_height(self, lift_height):
        self._lift_height = lift_height

    @property
    def PCF(self):
        return self._PCF

    @PCF.setter
    def PCF(self, PCF):
        self._PCF = PCF
        
    def _record_state(self, tag="update"):
        """Record the current frames of the mobile robot."""
        entry = {
            "time": time.time(),
            "tag": tag,
            "WCF_slam": self._frame_to_dict(self._WCF_slam),
            "BCF_slam": self._frame_to_dict(self._BCF_slam),
            "BCF":  self._frame_to_dict(self._BCF),
            "BCF_gt": self._frame_to_dict(self._BCF_gt)
        }
        self._frame_log.append(entry)
    
    def export_log(self, path):
        with open(path, "w") as f:
            json.dump(self._frame_log, f, indent=2)
    
    def _frame_to_dict(self, frame):
            if frame is None:
                return None
            return {
                "point": list(frame.point),
                "xaxis": list(frame.xaxis),
                "yaxis": list(frame.yaxis)
            }
        
    def transformation_BCF_WCF(self):
        """Get the transformation from the base coordinate frame (BCF) to the world coordinate frame (WCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_change_of_basis(self.BCF, Frame.worldXY())

    def transformation_WCF_BCF(self):
        """Get the transformation from the world coordinate frame (WCF) to the base coordinate frame (BCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_change_of_basis(Frame.worldXY(), self.BCF)

    def transformation_RCF_WCF(self):
        """Get the transformation from the robot arm coordinate frame (RCF) to the world coordinate frame (WCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.concatenated(
            self.transformation_BCF_WCF(), self.transformation_RCF_BCF()
        )

    def transformation_WCF_RCF(self):
        """Get the transformation from the world coordinate frame (WCF) to the robot arm coordinate frame (RCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.concatenated(
            self.transformation_BCF_RCF(), self.transformation_WCF_BCF()
        )

    def transformation_RCF_BCF(self):
        """Get the transformation from the robot arm frame (RCF) to the base coordinate frame (BCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_frame(self.RCF)

    def transformation_BCF_RCF(self):
        """Get the transformation from the base coordinate frame (BCF) to the robot arm frame (RCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_frame(self.RCF).inverted()

    def transformation_BCF_slam_WCF(self):
        """Get the transformation from the reference base coordinate frame (BCF_slam) to the world coordinate frame (WCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        frame_BCF_slam_in_WCF = self.WCF_slam.to_world_coordinates(self._BCF_slam)
        return Transformation.from_change_of_basis(frame_BCF_slam_in_WCF, Frame.worldXY())

    def transformation_WCF_BCF_slam(self):
        """Get the transformation from the world coordinate frame (WCF) to the reference base coordinate frame (BCF_slam).
        -------
        :class:`compas.geometry.Transformation`
        """
        frame_BCF_slam_in_WCF = self.WCF_slam.to_world_coordinates(self._BCF_slam)
        return Transformation.from_change_of_basis(Frame.worldXY(), frame_BCF_slam_in_WCF)

    def transformation_WCF_slam_WCF(self):
        """Get the transformation from the reference world coordinate frame (WCF_slam) to the world coordinate frame (WCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_change_of_basis(self.WCF_slam, Frame.worldXY())

    def transformation_WCF_WCF_slam(self):
        """Get the transformation from the world coordinate frame (WCF) to the reference world coordinate frame (WCF_slam).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_change_of_basis(Frame.worldXY(), self.WCF_slam)
    
    def transformation_WCF_slam_BCF_slam(self):
        """Get the transformation from the reference world coordinate frame (WCF_slam) to the reference base coordinate frame (BCF_slam).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_change_of_basis(self._WCF_slam, self._BCF_slam)

    def transformation_BCF_slam_WCF_slam(self):
        """Get the transformation from the reference base coordinate frame (BCF_slam) to the reference world coordinate frame (WCF_slam).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_change_of_basis(self._BCF_slam, self._WCF_slam)

    def from_WCF_to_BCF(self, frame_WCF):
        """Represent a frame from the world coordinate system (WCF) in the robot base coordinate system (BCF).
        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        Returns
        -------
        frame_BCF : :class:`compas.geometry.Frame`
            A frame in the robot base coordinate frame.
        """
        frame_BCF = frame_WCF.transformed(self.transformation_WCF_BCF())
        return frame_BCF

    def from_BCF_to_WCF(self, frame_BCF):
        """Represent a frame from the robot's base coordinate system (BCF) in the world coordinate system (WCF).
        Parameters
        ----------
        frame_BCF : :class:`compas.geometry.Frame`
            A frame in the robot base coordinate frame.
        Returns
        -------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        """
        frame_WCF = frame_BCF.transformed(self.transformation_BCF_WCF())
        return frame_WCF

    def from_WCF_to_RCF(self, frame_WCF):
        """Represent a frame from the world coordinate system (WCF) in the robot arm coordinate system (RCF).
        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        Returns
        -------
        frame_RCF : :class:`compas.geometry.Frame`
            A frame in the robot arm coordinate frame.
        """
        frame_RCF = frame_WCF.transformed(self.transformation_WCF_RCF())
        return frame_RCF

    def from_RCF_to_WCF(self, frame_RCF):
        """Represent a frame from the robot arm coordinate system (RCF) in the world coordinate system (WCF).
        Parameters
        ----------
        frame_RCF : :class:`compas.geometry.Frame`
            A frame in the robot arm coordinate frame.
        Returns
        -------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        """
        frame_WCF = frame_RCF.transformed(self.transformation_RCF_WCF())
        return frame_WCF

    def from_RCF_to_BCF(self, frame_RCF):
        """Represent a frame from the robot arm coordinate system (RCF) in the robot base coordinate system (BCF).
        Parameters
        ----------
        frame_RCF : :class:`compas.geometry.Frame`
            A frame in the robot arm coordinate frame.
        Returns
        -------
        frame_BCF : :class:`compas.geometry.Frame`
            A frame in the robot base coordinate frame.
        """
        frame_BCF = frame_RCF.transformed(self.transformation_RCF_BCF())
        return frame_BCF

    def from_BCF_to_RCF(self, frame_BCF):
        """Represent a frame from the robot base coordinate system (BCF) in the robot arm coordinate system (RCF).
        Parameters
        ----------
        frame_BCF : :class:`compas.geometry.Frame`
            A frame in the robot base coordinate frame.
        Returns
        -------
        frame_RCF : :class:`compas.geometry.Frame`
            A frame in the robot arm coordinate frame.
        """
        frame_RCF = frame_BCF.transformed(self.transformation_BCF_RCF())
        return frame_RCF

    def transform_frame_from_RCF_to_BCF(self, frame_WCF):
        """Apply the transformation between RCF and BCF to a frame."""
        inverted_RCF = Frame.from_transformation(self.transformation_BCF_RCF())
        return inverted_RCF.transformed(Transformation.from_frame(frame_WCF))

    def from_WCF_to_WCF_slam(self, frame_WCF):
        """Represent a frame from the world coordinate system (WCF) in the reference world coordinate system (WCF_slam).
        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        Returns
        -------
        :class:`compas.geometry.Frame`
            A frame in the robot's coordinate frame.
        """
        frame_WCF_slam = frame_WCF.transformed(self.transformation_WCF_WCF_slam())
        return frame_WCF_slam
    
    def from_WCF_slam_to_WCF(self, frame_WCF_slam):
        """Represent a frame from the world coordinate system (WCF) in the reference world coordinate system (WCF_slam).
        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        Returns
        -------
        :class:`compas.geometry.Frame`
            A frame in the robot's coordinate frame.
        """
        frame_WCF = frame_WCF_slam.transformed(self.transformation_WCF_slam_WCF())
        return frame_WCF
    
    def from_WCF_slam_to_BCF_slam(self, frame_WCF_slam):
        """Represent a frame from the reference world coordinate system (WCF_slam) in the reference base coordinate system (BCF_slam).
        Parameters
        ----------
        frame_WCF_slam : :class:`compas.geometry.Frame`
            A frame in the reference world coordinate frame.
        Returns
        -------
        :class:`compas.geometry.Frame`
            A frame in the reference base coordinate frame.
        """
        frame_BCF_slam = frame_WCF_slam.transformed(self.transformation_WCF_slam_BCF_slam())
        return frame_BCF_slam   
    
    def from_BCF_slam_to_WCF_slam(self, frame_BCF_slam):
        """Represent a frame from the reference base coordinate system (BCF_slam) in the reference world coordinate system (WCF_slam).
        Parameters
        ----------
        frame_BCF_slam : :class:`compas.geometry.Frame`
            A frame in the reference base coordinate frame.
        Returns
        -------
        :class:`compas.geometry.Frame`
            A frame in the reference world coordinate frame.
        """
        frame_WCF_slam = frame_BCF_slam.transformed(self.transformation_BCF_slam_WCF_slam())
        return frame_WCF_slam
    
