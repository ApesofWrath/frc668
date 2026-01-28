import limelight 
import limelightresults
import json
import time 
from websocket import create_connection, WebSocketException, WebSocketApp

class Vision:
    def __init__ (self):
        self.l1_address = "10.6.68.11"
        self.l1 = limelight.Limelight(self.l1_address)
        self.l1.pipeline_switch(0)
        self.l1.enable_websocket()
        self.websocketConnected = False
        self.websocketAttemptDelay = 0
        self.ws = None  
        self.m = "none"

    def connectWebsocket(self):
        self.m = "started"
        try:
            self.ws = create_connection(f"ws://{self.l1_address}:5806")
            self.websocketConnected = True
            print("Websocket connected")
            self.m = "connected"
        except Exception as e:
            self.websocketConnected = False
            print("Websocket failed to connect: ")
            print(e)
            self.m = "fail"
        if self.m == "started":
            self.m = "nope"
    
    def execute(self) -> None:
        if self.websocketAttemptDelay <= 0: 
            if not self.websocketConnected: 
                self.websocketAttemptDelay = 20
                self.connectWebsocket()
            else:
                self.websocketAttemptDelay -= 1
        print(self.m)
        if self.ws is not None:
            if self.ws.connected:
                self.receiveMessage(self.ws.recv())
        
    def receiveMessage(self,_data) -> None:
        if _data is None:
            return
        data = json.loads(_data)
        print(data)
        if data.get("v") == 1:
            fiducial = data.get("Fiducial", {})
            print("Fiducial Data: ", fiducial)
            print("botpose_orb_wpiblue: ", data.get("botpose_orb_wpiblue"))
        # get location based off of data and do things

    