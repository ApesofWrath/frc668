import limelight 
import limelightresults
import json
import time 
from websocket import create_connection, WebSocketException, WebSocketApp

class Vision:
    def __init__ (self):
        l1_address = "10.6.68.11"
        self.l1 = limelight.Limelight(l1_address)
        self.l1.enable_websocket()
        # self.websocketConnected = False
        # self.websocketAttemptDelay = 0
        self.ws = WebSocketApp("ws://10.6.68.11:5806",
                              on_open=self.on_open,
                              on_message=self.on_message,
                              on_error=self.on_error,
                              on_close=self.on_close)
        self.ws.run_forever()
        

    '''def connectWebsocket(self):
        try:
            self.ws = create_connection("ws://10.6.68.11:5806")
            self.websocketConnected = True
            print("websocket connected")
        except Exception as e:
            self.websocketConnected = False
            print("websocket failed to connect connected")
            print(e)
    '''

    def stream_results(self):
        
        """
        print("Connected:", self.l1.websocket_connected)
        result = self.l1.get_latest_results()
        parsed_result = limelightresults.parse_results(result)
        """    

    def execute(self)->None:
        """
        if self.websocketAttemptDelay <= 0: 
            if not self.websocketConnected: 
                self.connectWebsocket()
            else:
                self.websocketAttemptDelay -= 1
        """
        

    # def stream_results(self):
    #     packet = self.l1.get_latest_results()
    #     print("valid targets: ", packet.validity, ", pipelineIndex: ", packet.pipeline_id,", Targeting Latency: ", packet.targeting_latency)
    #     if packet is not None:
    #         for tag in packet.fiducialResults:
    #             print(tag.robot_pose_target_space, tag.fiducial_id)
    #         time.sleep(1)
    #     else:
    #         print("parsed_result is none")

    def on_message(ws, message):
        print(f"Received: {message}")
    def on_open(ws):
        print("Webcsocket opened")
    def on_error(ws, error):
        print(f"Error: {error}")
    def on_close(ws, close_status_code, close_msg):
        print("websocket closed :(")
