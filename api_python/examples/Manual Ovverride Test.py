import ObjDetectionTest1

# First import the library
import pyrealsense2 as rs


import time
import sys
import os
import threading

sys.path.insert(0, '../106-Gripper_command/01-gripper_command')

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2

from playsound import playsound 
from gtts import gTTS 

def stopMovement(base):

    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0
    time.sleep(.5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

 

    return True



def main():
    import argparse

    
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)

        # Example core
           
        print("Returning Arm to Home")
        stopMovement(base)



        print("End")




    #ObjDetectionTest1.basicOBJDetect() comment back in once coordinates are figured out, might have to change coordinates within objdetection file as well

if __name__ == "__main__":
    exit(main())