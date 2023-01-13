from compas_fab.robots import Robot

from compas.geometry import Frame
from compas.geometry import Transformation

__all__ = [
    "MobileRobot"
]

class MobileRobot(Robot):
    """Represents a robot, which can be moved in the world coordinate system.
    """
    def __init__(self, model, artist=None, semantics=None, client=None):
        super(Robot, self).__init__(model, artist, semantics, client)

        self._scale_factor = 1.
        self.model = model
        # self.attached_tool = None
        self.artist = artist
        self.semantics = semantics
        self.client = client
        self.attributes = {}
        self._current_ik = {
            'request_id': None,
            'solutions': None
        }

        self._reference_frame = Frame.worldXY()
        # robot reference frame (REF)

        self._robot_coordinate_frame = Frame.worldXY()
        # robot coordinate frame in reference frame (RCF)

        self._picking_frame = Frame.worldXY()
        # frame for element pick-up on mobile robot's base

    @property
    def reference_frame(self):
        """Get the robot's reference frame.
        :class:`compas.geometry.Frame`
        """
        return self._reference_frame
    
    @reference_frame.setter
    def reference_frame(self, reference_frame):
        """Move the reference frame of the robot to the robot_coordinate_frame.
        """
        self._reference_frame = reference_frame

    @property
    def robot_coordinate_frame(self):
        return self._robot_coordinate_frame

    @robot_coordinate_frame.setter
    def robot_coordinate_frame(self, robot_coordinate_frame):
        self._robot_coordinate_frame = robot_coordinate_frame

    def transformation_RCF_WCF(self):
        """Get the transformation from the robot's reference frame (RCF) to the world coordinate frame (WCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        frame_RCF_in_WCF = self.reference_frame.to_world_coordinates(self.robot_coordinate_frame)
        return Transformation.from_change_of_basis(frame_RCF_in_WCF, Frame.worldXY())

    def transformation_REF_WCF(self):
        """Get the transformation from the robot's reference frame (REF) to the world coordinate frame (WCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_change_of_basis(self.reference_frame, Frame.worldXY())

    def transformation_WCF_RCF(self):
        """Get the transformation from the world coordinate frame (WCF) to the robot's reference frame (RCF).
        -------
        :class:`compas.geometry.Transformation`
        """
        frame_RCF_in_WCF = self.reference_frame.to_world_coordinates(self.robot_coordinate_frame)
        return Transformation.from_change_of_basis(Frame.worldXY(), frame_RCF_in_WCF)

    def transformation_WCF_REF(self):
        """Get the transformation from the world coordinate frame (WCF) to the robot's reference frame (REF).
        -------
        :class:`compas.geometry.Transformation`
        """
        return Transformation.from_change_of_basis(Frame.worldXY(), self.reference_frame)

    def to_local_coordinates(self, frame_WCF):
        """Represent a frame from the world coordinate system (WCF) in the robot coordinate system (RCF).
        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        Returns
        -------
        :class:`compas.geometry.Frame`
            A frame in the robot's coordinate frame.
        """
        frame_RCF = frame_WCF.transformed(self.transformation_WCF_RCF())
        return frame_RCF

    def to_reference_coordinates(self, frame_WCF):
        """Represent a frame from the world coordinate system (WCF) in the robot's reference coordinate system (REF).
        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        Returns
        -------
        :class:`compas.geometry.Frame`
            A frame in the robot's coordinate frame.
        """
        frame_REF = frame_WCF.transformed(self.transformation_WCF_REF())
        return frame_REF

    def to_world_coordinates(self, frame_RCF):
        """Represent a frame from the robot's reference coordinate system (REF) in the world coordinate system (WCF).
        Parameters
        ----------
        frame_OCF : :class:`compas.geometry.Frame`
            A frame in the robot's coordinate frame.
        Returns
        -------
        :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        """
        frame_WCF = frame_RCF.transformed(self.transformation_RCF_WCF())
        return frame_WCF
    
    @property
    def picking_frame(self):
        return self._picking_frame
    
    @picking_frame.setter
    def set_picking_frame(self, picking_frame):
        self._picking_frame = picking_frame