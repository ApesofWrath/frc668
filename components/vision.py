import ntcore
import constants
from magicbot import feedback
from components.limelight import LimelightHelpers, PoseEstimate 
from phoenix6 import hardware
from phoenix6.swerve import swerve_drivetrain
from phoenix6 import units
import math
from typing import Any
from wpilib import DriverStation
from wpilib import geometry


class Vision:
    drivetrain: swerve_drivetrain
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

    
    def start_match (self) -> None:
        """
        Resets the pigeon yaw at the start of the match based on alliance color.
        This is necessary to ensure that the robot's orientation is correct for vision 
        processing and field-relative driving.
        """
        alliance = DriverStation.getAlliance()
        if alliance == DriverStation.Alliance.kBlue:
            self.pigeon.set_yaw(0, 0.1)
        elif alliance == DriverStation.Alliance.kRed:
            self.pigeon.set_yaw(180, 0.1)
        else:
            self.pigeon.set_yaw(0,0.1)
            self.logger.info("Warning: Alliance not found at start: Defaulting to Blue (0 degrees)")


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

        self.mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(limelight_name)
        return self.mega_tag2

   
    def mega_tag2_logic(self, limelight_name: str) -> tuple[bool, str]:
        """
        Validates MegaTag2 vision estimates using three rejection criteria:
        1. Rejects single-tag detections at distances > 4 meters.
        2. Rejects estimates > 2 meters away from current odometry pose.
        3. Rejects estimates outside field bounds (X: 0-16.54m, Y: 0-8.05m).
        """
        # Calculate distance once to save processing/readability
        odo_pose = self.drivetrain.get_state().pose
        distance_error = odo_pose.translation().distance(self.mega_tag2.pose.translation())

        if self.mega_tag2 is None or self.mega_tag2.tag_count == 0:
            return False, "No tags detected"
        
        if self.mega_tag2.tag_count == 1 and self.mega_tag2.avg_tag_dist > 4.0:
            return False, "Too far from 1 tag"
            
        if distance_error > 2.0:
            return False, "Odometry and vision differ too much"

        # If the pose is NOT within the X range OR NOT within the Y range, reject.
        if not (0.0 < self.mega_tag2.pose.X < 16.54 and 0.0 < self.mega_tag2.pose.Y < 8.05):
            return False, "Pose estimate is outside of field boundaries"

        return True, ""

    
    def field_location(self) -> geometry.Pose2d | None:
        """
        Main method for determining field-relative pose using MegaTag2.
        """
        is_valid, reason = self.mega_tag2_logic(self.l1)
        if is_valid:
            dist = self.mega_tag2.avg_tag_dist 
            stdDevs = (
                0.1 + dist * 0.05,
                0.1 + dist * 0.05, 
                0.2 + dist * 0.1       
            )
            self.drivetrain.addVisionMeasurement(
                self.mega_tag2.pose,
                self.mega_tag2.timestamp_seconds * units.second,
                stdDevs
            )
            return self.drivetrain.get_state().pose

   


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

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    """
    Code commented out has no use now but could later
    """
            
        # self.botpose_wpiblue_topic = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe([0.0,0.0,0.0,0.0,0.0,0.0])
        # self.tid_topic = table.getIntegerTopic("tid").subscribe(0)
        # self.camerapose_targetspace_topic = table.getDoubleArrayTopic('camerapose_targetspace').subscribe([0.0,0.0,0.0,0.0,0.0,0.0])
    
            # new_table = ntcore.NetworkTableInstance.getDefault().getTable("components").getSubTable("vision")
        # self.pubm2 = new_table.getDoubleArrayTopic("pose_estimate_m2").publish()
        # self.pubm1 = new_table.getDoubleArrayTopic("pose_estimate_m1").publish()

        # if DriverStation.getAlliance==DriverStation.Alliance.kBlue:
        #     self.pigeon.reset()
        # elif DriverStation.getAlliance==DriverStation.Alliance.kRed:
        #     self.pigeon.set_yaw(180)
    
    
        #     self.pubm2.set(self.get_pose_estimate_m2(self.l1))

    # @feedback
    # def get_tid(self):
    #     return self.tid_topic.get()

    # @feedback
    # def get_camerapose_targetspace(self):
    #     return self.camerapose_targetspace_topic.get()

    # @feedback
    # def get_botpose_orb_wpiblue(self):
    #     return self.botpose_wpiblue_topic.get()
    
    
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
