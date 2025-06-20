import numpy as np


class MarkerSet:
    """
    A class to manage a set of markers
    """
    def __init__(self, marker_type):
        """
        Initialize the marker set with a specific type of marker
        :param marker_type: The type of marker to use (e.g., 'sphere', 'cube')
        """
        self.marker_type = marker_type
        self.markers = {}
    
    def add_marker(self, marker_id, position, orientation, robot_frame=None):
        """
        Add a marker to the set
        :param marker_id: Unique identifier for the marker
        :param position: Position of the marker in 3D space (x, y, z)
        :param orientation: Orientation of the marker (roll, pitch, yaw)
        """
        self.markers[marker_id] = {
            'position': position,
            'orientation': orientation,
            'robot_frame': robot_frame,
        }
    
    def add_markers(self, markers):
        """
        Add multiple markers to the set
        :param markers: A dictionary of markers with their IDs, positions, and orientations
        """
        for marker_id, marker_data in markers.items():
            self.add_marker(marker_id, marker_data['position'], marker_data['orientation'], marker_data.get('robot_frame'))
    
    @classmethod
    def from_data(cls, data):
        """
        Populate the marker set from a data source
        :param data: Data source containing marker information
        """
        marker_set = cls(data['marker_type'])
        for marker_id, marker_info in data.items():
            if marker_id == 'marker_type':
                continue
            position = marker_info['position']
            orientation = marker_info['orientation']
            robot_frame = marker_info.get('robot_frame', None)
            marker_set.add_marker(marker_id, position, orientation, robot_frame)
        return marker_set

    def to_data(self):
        """
        Convert the marker set to a data format suitable for storage or transmission
        :return: A dictionary representation of the marker set
        """
        data = {'marker_type': self.marker_type}
        for marker_id, marker_info in self.markers.items():
            data[marker_id] = {
                'position': marker_info['position'],
                'orientation': marker_info['orientation'],
                'robot_frame': marker_info.get('robot_frame', None),
            }
        return data