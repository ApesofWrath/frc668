import ntcore
import constants
from magicbot import feedback
from components.limelight import LimelightHelpers, PoseEstimate 
from phoenix6 import hardware
import math
from typing import Any

class Vision:

    logger: Any

    def __init__(self):
        self.l1 = constants.LIMELIGHT_ONE
        table = ntcore.NetworkTableInstance.getDefault().getTable(self.l1)
        self.botpose_wpiblue_topic = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe([0.0,0.0,0.0,0.0,0.0,0.0])
        self.tid_topic = table.getIntegerTopic("tid").subscribe(0)
        self.camerapose_targetspace_topic = table.getDoubleArrayTopic('camerapose_targetspace').subscribe([0.0,0.0,0.0,0.0,0.0,0.0])
        self.pigeon = hardware.Pigeon2(constants.PIGEON_ID, "rio")

    def execute(self) -> None:
        #self.set_orientation(self.l1)
        self.test()
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

    def set_orientation(self, limelight_name):
        orientation = self.pigeon.getRotation3d()
        LimelightHelpers.set_robot_orientation(limelight_name, orientation.Z()*180.0/math.pi, 0, orientation.Y()*180.0/math.pi, 0, orientation.X()*180.0/math.pi, 0)
        self.logger.info("orientation success")
        mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(self.l1)
        self.logger.info("botpose success")

    @feedback
    def get_pose_estimate(self):
        orientation = self.pigeon.getRotation3d()
        LimelightHelpers.set_robot_orientation(self.l1, orientation.Z()*180.0/math.pi, 0, orientation.Y()*180.0/math.pi, 0, orientation.X()*180.0/math.pi, 0)
        self.logger.info("orientation success")
        mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(self.l1)
        self.logger.info("botpose success")
        ret = [mega_tag2.pose.X(), mega_tag2.pose.Y(), mega_tag2.pose.rotation().degrees()]
        for item in mega_tag2.raw_fiducials:
            ret.append(item.id)
        return ret 
    