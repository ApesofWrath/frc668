import limelight
import limelightresults
import json
import time

class Vision:
    def __init__ (self):
        self.discovered_limelights = limelight.discover_limelights(debug=True)
        print("discovered limelights:", self.discovered_limelights)
    

    ll = limelight.Limelight()

