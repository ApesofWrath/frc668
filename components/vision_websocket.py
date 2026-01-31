import ntcore
import constants
from magicbot import feedback
from components.limelight import LimelightHelpers, PoseEstimate 
from phoenix6 import hardware
import math

class Vision:

    def __init__(self):
        self.l1 = "limelight-one"
        table = ntcore.NetworkTableInstance.getDefault().getTable(self.l1)
        self.botpose_wpiblue_topic = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe([0.0,0.0,0.0,0.0,0.0,0.0])
        self.tid_topic = table.getIntegerTopic("tid").subscribe(0)
        self.camerapose_targetspace_topic = table.getDoubleArrayTopic('camerapose_targetspace').subscribe([0.0,0.0,0.0,0.0,0.0,0.0])
        self.pigeon = hardware.Pigeon2(constants.PIGEON_ID, "rio")

   
    def execute(self) -> None:
        pass 

    @feedback
    def get_tid(self):
        return self.tid_topic.get()

    @feedback
    def get_camerapose_targetspace(self):
        return self.camerapose_targetspace_topic.get()

    @feedback
    def get_botpose_orb_wpiblue(self):
        return self.botpose_wpiblue_topic.get()

    @feedback
    def get_orientation(self):
        orientation = self.pigeon.getRotation3d()
        return [orientation.X(),orientation.Y(),orientation.Z()]

    # @feedback
    # def get_pose_estimate(self):
    #     orientation = self.pigeon.getRotation3d()
    #     LimelightHelpers.set_robot_orientation(self.l1, orientation.Z()*180.0/math.pi, 0, orientation.Y()*180.0/math.pi, 0, orientation.X()*180.0/math.pi, 0)
    #     #TODO: fill in yaw, pitch, and roll rates 
    #     mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(self.l1)
    #     if mega_tag2.tag_count > 0:
    #         return mega_tag2
    #     return None