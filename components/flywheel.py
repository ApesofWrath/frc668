
import constants
import components.vision as vision
import numpy
from typing import Any
import magicbot

class Flywheel:
    logger: Any

    def __init__(self):
        data = numpy.array([
            [0,0,0]
            #TODO: add data on distance between robot pose and hub, hood angle, flywheel velocity
        ])

        self.distances = data[:, 0] 
        self.hood_angle = data[:, 1] 
        self.flywheel_velocity = data[:, 2]

    def execute(self) -> None:  
        current_distance = magicbot.tunable(0.0)
        target_hood_angle = magicbot.tunable(0.0)
        target_flywheel_velocity = magicbot.tunable(0.0)

        #TODO: get current_distance from robot pose using all chassis limelights  
    
        #target_hood_angle = numpy.interp(current_distance, self.distances, self.hood_angle)
        #target_flywheel_velocity = numpy.interp(current_distance, self.distances, self.flywheel_velocity)

        self.logger.info("Distance: ", current_distance)
        self.logger.info("Target hood angle: ", target_hood_angle)
        self.logger.info("Target flywheel velocity: ", target_flywheel_velocity)