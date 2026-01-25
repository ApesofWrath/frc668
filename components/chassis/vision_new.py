import limelight 
import limelightresults
import json
import time 
from websocket import create_connection, WebSocketException

class Vision:
    def __init__ (self):
        l1_address = "10.6.68.11"
        self.l1 = limelight.Limelight(l1_address)
        self.l1.enable_websocket()
        self.websocket_sucsess = "not yet"
        self.websocket_tested = False
        

    def websocket_test(self):
        self.websocket_sucsess = "started..."
        try:
            self.ws = create_connection("ws://10.6.68.11:5806")
            self.websocket_sucsess = "yes"
            print("websocket connected")
            self.ws.close()
        except Exception as e:
            print(e)
            self.websocket_sucsess = f"no!!! {e}"
            print("websocket not connected")
        if self.websocket_sucsess == "started...":
            self.websocket_sucsess = "nothing"

    def stream_results(self):
        
        """
        print("Connected:", self.l1.websocket_connected)
        result = self.l1.get_latest_results()
        parsed_result = limelightresults.parse_results(result)
        if parsed_result is not None:
            print("valid targets: ", parsed_result.validity, ", pipelineIndex: ", parsed_result.pipeline_id,", Targeting Latency: ", parsed_result.targeting_latency)
            for tag in parsed_result.fiducialResults:
                print(tag.robot_pose_target_space, tag.fiducial_id)
            time.sleep(1)
        else:
            print("parsed_result is none")  
        """    

    def execute(self)->None:
        if not self.websocket_tested:
            self.websocket_tested = True
            self.websocket_test()
        print(self.websocket_sucsess)