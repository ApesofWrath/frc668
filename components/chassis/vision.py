import ntcore
import constants
from magicbot import feedback
from components.limelight import LimelightHelpers, PoseEstimate 
from phoenix6 import hardware
from phoenix6 import units
import math
from typing import Any
from wpilib import DriverStation
from wpilib import geometry


class Vision:
    logger: Any

    def __init__(self):
        self.l1 = constants.LIMELIGHT_ONE
        self.pigeon = hardware.Pigeon2(constants.PIGEON_ID, "rio")
        
        table = ntcore.NetworkTableInstance.getDefault().getTable(self.l1)
        self.pose_diff_pub = table.getFloatTopic('Vision vs Odometry Pose Difference').publish()
        self.reject_pub = table.getStringTopic('Rejection Reason').publish()
        self.timestamp_pub = table.getFloatTopic('Last Good Timestamp').publish()

    def execute(self) -> None:
        pass


    def set_robot_pose_m2 (self,limelight_name) -> None:
        """
        Sets the limelight's robot orientation using the pigeon
        """
        orientation = self.pigeon.getRotation3d()
        
        LimelightHelpers.set_robot_orientation(
            limelight_name,
            math.degrees(orientation.Z()), 0,
            math.degrees(orientation.Y()), 0,
            math.degrees(orientation.X()), 0
        )

        
    def get_mega_tag2_pose(self, limelight_name: str) -> PoseEstimate:    
        self.mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(limelight_name)
        mega_tag2_rotation = LimelightHelpers.get_botpose_3d_wpiblue(limelight_name)

        if self.mega_tag2 is None or self.mega_tag2.tag_count == 0:
            return 
        
        if self.mega_tag2.tag_count == 1 and self.mega_tag2.avg_tag_dist > 4.0:
            return 

        if self.mega_tag2_rotation.rotation().X() != 0.0:
            return 

        # If the pose is NOT within the X range OR NOT within the Y range, reject.
        if not (0.0 < self.mega_tag2.pose.X < 16.54 and 0.0 < self.mega_tag2.pose.Y < 8.05):
            return

        return self.mega_tag2


    
   


    @feedback
    def update_vision_diagnostics(self):
        """
        Runs once per loop to update all NetworkTable topics efficiently.
        """ 
        if self.mega_tag2 is None or self.mega_tag2.tag_count == 0:
            self.reject_pub.set("No tags detected")
            self.pose_diff_pub.set(0.0)
            return "No tags detected"

        self.reject_pub.set(reason if not is_valid else "Valid")

        diff = self.mega_tag2.pose.translation().distance(
            self.drivetrain.get_state().pose.translation()
        )
        self.pose_diff_pub.set(diff)

        if is_valid:
            self.timestamp_pub.set(self.mega_tag2.timestamp_seconds)
            
        return reason
