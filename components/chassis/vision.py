import limelight
import limelightresults
import json
import time

class Vision:
    def __init__ (self):
        try:
            self.limelight1_address = "10.6.68.11"                
            self.ll = limelight.Limelight(self.limelight1_address)
            self.ll.enable_websocket()
            results_url = f"http://{self.limelight1_address}:5807/results"

        except:
            print("Vision __init__ error")

    """
    def get_results(self):
        try:
                
            self.results = self.ll.get_results()
            self.status = self.ll.get_status()
                
            print("-----")
            print("targeting results:", self.results)
            print("-----")
            print("status:", self.status)
            print("-----")
            print("temp:", self.ll.get_temp())
            print("-----")
            print("name:", self.ll.get_name())
            print("-----")
            print("fps:", self.ll.get_fps())
            print("-----")
            print("hwreport:", self.ll.hw_report())

        except:
            print("Vision get_results error")
    """
    def stream_results(self):
        """
        TO DO
        How to analyze stream data
        """
        try:
            while True:
                result = self.ll.get_latest_results()
                parsed_result = self.ll.limelight.parse_results(result)
                print("Validity", parsed_result.validity)
                print("ID", parsed_result.pipeline_id)
                print("FPS", parsed_result.fps)
                print("Latency", parsed_result.targeting_latency)
                print("Count", parsed_result.tag_count)
        except:
            print("Vision stream_results error")        

    

            
            """
                self.ll.enable_websocket()   
                
                print(self.ll.get_pipeline_atindex(0))

                # update the current pipeline and flush to disk
                pipeline_update = {
                'area_max': 98.7,
                'area_min': 1.98778
                }
                self.ll.update_pipeline(json.dumps(pipeline_update),flush=1)

                print(self.ll.get_pipeline_atindex(0))

                # switch to pipeline 1
                self.ll.pipeline_switch(1)

                # update custom user data
                self.ll.update_python_inputs([4.2,0.1,9.87])

        except:
            print("Vision __init__ error")
        
    
    def stream_results(self):
        
        Continuously reads results from Limelight

        try:
            while True:
                result = self.ll.get_latest_results()
                parsed_result = limelightresults.parse_results(result)
                if parsed_result is not None:
                    print("valid targets: ", parsed_result.validity, ", pipelineIndex: ", parsed_result.pipeline_id,", Targeting Latency: ", parsed_result.targeting_latency)
                    #---for tag in parsed_result.fiducialResults:
                    #   print(tag.robot_pose_target_space, tag.fiducial_id)
                    time.sleep(1)      
        
        except KeyboardInterrupt:
            print("Program interrupted by user, shutting down.")
        finally:
            self.ll.disable_websocket()


class GeneralResult:


"""



    