from compas_fab.robots import Robot

from compas.geometry import Frame, Point, Vector
from compas.geometry import Transformation, Translation, Quaternion
from roslibpy import Message, Topic, Service, tf

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
        self.attributes = {}
        self._current_ik = {"request_id": None, "solutions": None}

        self._lift_height = 0  # lift height

        self._WCF = Frame.worldXY()  # world coordinate frame (WCF)
        self._BCF = Frame.worldXY()  # base coordinate frame in WCF (BCF)
        self._RCF = None  # ur robot arm coordinate frame in BCF (RCF)
        self._RCF_lift = None  # ur robot arm coordinate frame in BCF (RCF) with lift height

        self._RWCF = Frame.worldXY()  # reference world coordinate frame (RWCF)
        self._RBCF = Frame.worldXY()  # base coordinate frame in RWCF (RBCF)
        self._RRCF = None  # ur robot arm coordinate frame in RBCF (RRCF)

        self._PCF = (
            Frame.worldXY()
        )  # frame for element pick-up on mobile robot's base in RCF (PCF)

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

    @property
    def BCF(self):
        return self._BCF

    @BCF.setter
    def BCF(self, BCF):
        self._BCF = BCF

    @property
    def RCF(self):
        if self.mobile_client is not None:
            if self._RCF is None:
                self.mobile_client.tf_subscribe(
                    "robot_arm_base",
                    "robot_base_footprint",
                    self._receive_base_frame_callback,
                    timeout=5,
                )
            if self.lift_height != 0:
                self._RCF_lift = self._RCF.copy()
                self._RCF_lift.point.z += self.lift_height
            else:
                self._RCF_lift = self._RCF
        return self._RCF_lift

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

        # if self._RCF == None:
        #     robot_arm_base_link = self.forward_kinematics(self.zero_configuration(), 'ur10e', True, options={'link':'robot_arm_base_link'})
        #     self._RCF = Frame(robot_arm_base_link.point, -robot_arm_base_link.xaxis, -robot_arm_base_link.yaxis)
        # if self.wheel_type == 'outdoor':
        #     self._RCF = Frame(Point(0.275, 0.0, 1.049 + self.lift_height), Vector(-0.707, 0.707, 0.0), Vector(-0.707, -0.707, 0.0))
        # elif self.wheel_type == 'indoor':
        # self._RCF = Frame(Point(0.275, 0.0, 1.021 + self.lift_height), Vector(-0.707, 0.707, 0.0), Vector(-0.707, -0.707, 0.0))
        return self._RCF

    @property
    def RWCF(self):
        """Get the reference world coordinate frame.
        :class:`compas.geometry.Frame`
        """
        return self._RWCF

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

    def transformation_RBCF_WCF(self):
        """Get the transformation from the reference base coordinate frame (RBCF) to the world coordinate frame (WCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        frame_RBCF_in_WCF = self.RWCF.to_world_coordinates(self._RBCF)
        return Transformation.from_change_of_basis(frame_RBCF_in_WCF, Frame.worldXY())

    def transformation_WCF_RBCF(self):
        """Get the transformation from the world coordinate frame (WCF) to the reference base coordinate frame (RBCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        frame_RBCF_in_WCF = self.RWCF.to_world_coordinates(self._RBCF)
        return Transformation.from_change_of_basis(Frame.worldXY(), frame_RBCF_in_WCF)

    def transformation_RWCF_WCF(self):
        """Get the transformation from the reference world coordinate frame (RWCF) to the world coordinate frame (WCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_change_of_basis(self.RWCF, Frame.worldXY())

    def transformation_WCF_RWCF(self):
        """Get the transformation from the world coordinate frame (WCF) to the reference world coordinate frame (RWCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_change_of_basis(Frame.worldXY(), self.RWCF)

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

    def from_WCF_to_RWCF(self, frame_WCF):
        """Represent a frame from the world coordinate system (WCF) in the reference world coordinate system (RWCF).
        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        Returns
        -------
        :class:`compas.geometry.Frame`
            A frame in the robot's coordinate frame.
        """
        frame_RWCF = frame_WCF.transformed(self.transformation_WCF_RWCF())
        return frame_RWCF
