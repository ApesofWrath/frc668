import ntcore
import constants
from magicbot import feedback
from components.limelight import LimelightHelpers, PoseEstimate 
from phoenix6 import hardware

class Vision:

    def __init__(self):
        self.l1 = "limelight-one"
        table = ntcore.NetworkTableInstance.getDefault().getTable(self.l1)
        self.newTable = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe([0,0,0,0,0,0])
        self.pigeon = hardware.Pigeon2(constants.PIGEON_ID, "rio")
    
    def execute(self) -> None:
        pass 

    @feedback
    def get_botpose_orb_wpiblue(self):
        return self.newTable.get()
    
    @feedback
    def get_pose_estimate(self):
        LimelightHelpers.set_robot_orientation(self.l1, self.pigeon.get_yaw(), 0, self.pigeon.get_pitch(), 0, self.pigeon.get_roll(), 0)
        #TODO: fill in yaw, pitch, and roll rates 
        mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(self.l1)
        if mega_tag2.tag_count > 0:
            return mega_tag2
        return None