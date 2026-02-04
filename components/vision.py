import ntcore
import constants
from magicbot import feedback
from components.limelight import LimelightHelpers, PoseEstimate 
from phoenix6 import hardware
import math
from typing import Any
from wpilib import DriverStation
from phoenix6.swerve import swerve_drivetrain

class Vision:

    logger: Any

    def __init__(self):
        self.l1 = constants.LIMELIGHT_ONE
        # table = ntcore.NetworkTableInstance.getDefault().getTable(self.l1)
        # self.botpose_wpiblue_topic = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe([0.0,0.0,0.0,0.0,0.0,0.0])
        # self.tid_topic = table.getIntegerTopic("tid").subscribe(0)
        # self.camerapose_targetspace_topic = table.getDoubleArrayTopic('camerapose_targetspace').subscribe([0.0,0.0,0.0,0.0,0.0,0.0])
        self.pigeon = hardware.Pigeon2(constants.PIGEON_ID, "rio")

        # new_table = ntcore.NetworkTableInstance.getDefault().getTable("components").getSubTable("vision")
        # self.pubm2 = new_table.getDoubleArrayTopic("pose_estimate_m2").publish()
        # self.pubm1 = new_table.getDoubleArrayTopic("pose_estimate_m1").publish()

        # if DriverStation.getAlliance==DriverStation.Alliance.kBlue:
        #     self.pigeon.reset()
        # elif DriverStation.getAlliance==DriverStation.Alliance.kRed:
        #     self.pigeon.set_yaw(180)

    def execute(self) -> None:
    #     self.pubm2.set(self.get_pose_estimate_m2(self.l1))
        pass 

    # @feedback
    # def get_tid(self):
    #     return self.tid_topic.get()

    # @feedback
    # def get_camerapose_targetspace(self):
    #     return self.camerapose_targetspace_topic.get()

    # @feedback
    # def get_botpose_orb_wpiblue(self):
    #     return self.botpose_wpiblue_topic.get()


    
    def start_match (self):
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self.pigeon.set_yaw(0)
        elif DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.pigeon.set_yaw(180)

    def get_pose_estimate_m2_v2(self,limelight_name):
        orientation = self.pigeon.getRotation3d()
        LimelightHelpers.set_robot_orientation(
            limelight_name,
            orientation.Z()*180.0/math.pi, 0,
            orientation.Y()*180.0/math.pi, 0,
            orientation.X()*180.0/math.pi, 0
        )
        mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(limelight_name)
        return mega_tag2

    def field_location(self):
        est = self.get_pose_estimate_m2_v2(self.l1)
        if est is None or est.tag_count == 0:
            return
        swerve_drivetrain.addVisionMeasurement(
            est.pose,
            est.timestamp,  
        )

@feedback
def get_orientation(self):
    o = self.pigeon.getRotation3d()
    return [
        math.degrees(o.X()),
        math.degrees(o.Y()),
        math.degrees(o.Z())
    ]


    # def get_pose_estimate_m2(self, limelight_name) -> list[float]:
    #     orientation = self.pigeon.getRotation3d()
    #     LimelightHelpers.set_robot_orientation(limelight_name, orientation.Z()*180.0/math.pi, 0, orientation.Y()*180.0/math.pi, 0, orientation.X()*180.0/math.pi, 0)
    #     self.logger.info("orientation success")
    #     mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(limelight_name)
    #     self.logger.info("botpose success - mega tag 2")
    #     ret = [mega_tag2.pose.X(), mega_tag2.pose.Y(), mega_tag2.pose.rotation().degrees()]
    #     for item in mega_tag2.raw_fiducials:
    #         ret.append(item.id)
    #     return ret

    # def get_pose_estimate_m1(self, limelight_name):
    #     mega_tag1 = LimelightHelpers.get_botpose_estimate_wpiblue(limelight_name)
    #     print("mega tag 1")
    #     ret = [mega_tag1.pose.X(), mega_tag1.pose.Y(), mega_tag1.pose.rotation().degrees()]
    #     for item in mega_tag1.raw_fiducials:
    #         ret.append(item.id)
    #     return ret