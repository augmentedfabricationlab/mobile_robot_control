from compas_fab.robots import Tool


class MultiTool(Tool):
    def __init__(self, visual, tool_frames, primary_tool_name="main", collision=None, name="attached_tool", connected_to=None):
        self.primary_tool_name = primary_tool_name
        self.tool_frames = tool_frames
        frame_in_tool0_frame = tool_frames.get(primary_tool_name)
        super(MultiTool, self).__init__(visual, frame_in_tool0_frame, collision, name, connected_to)   

    
    def set_active_tool_frame(self, tool_name="main"):
        frame_in_tool0_frame = self.tool_frames.get(tool_name, None)
        if frame_in_tool0_frame is None:
            raise KeyError("tool_name not recognized in MultiTool, please ensure tool_name:tool_frame is available in Multitool.tool_frames")
       
        self.tool_model.frame = frame_in_tool0_frame
        return frame_in_tool0_frame
    
    def add_tool_frame(self, tool_frame, tool_name):
        self.tool_frames.update({tool_name:tool_frame})