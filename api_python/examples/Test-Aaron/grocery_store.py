## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#####################################################
## librealsense tutorial #1 - Accessing depth data ##
#####################################################

# First import the library
import pyrealsense2 as rs
import cv2 
import multiprocessing
from realsense_camera import* 


import time
import sys
import os
import threading

sys.path.insert(0, '../106-Gripper_command/01-gripper_command')


from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2

from playsound import playsound 
from gtts import gTTS 

#Load Realsese camera 
rs = RealsenseCamera() 

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check


def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Serving mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        sys.exit(0)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    
    base.ExecuteActionFromReference(action_handle)
    
    # Leave time to action to complete
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def basicMovement(base, linX, linY, linZ, angX, angY, angZ):

    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    twist.linear_x = linX   #keep as .025 in main
    twist.linear_y = linY   # -.04
    twist.linear_z = linZ   #.03 
    twist.angular_x = angX
    twist.angular_y = angY
    twist.angular_z = angZ

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)
    

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)


    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

 

    return True

def SecondMovement(base):

    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    twist.linear_x = 0
    twist.linear_y = -0.0
    twist.linear_z = 0
    twist.angular_x = 0
    twist.angular_y = 0
    twist.angular_z = 0

    print("Beginning second movement...")
    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True

def X_Movement_Right(base):

    print("Moving to the right on the X axis")
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    twist.linear_x = 0.001
    twist.linear_y = -0.0
    twist.linear_z = 0
    twist.angular_x = 0
    twist.angular_y = 0
    twist.angular_z = 0

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True

def X_Movement_Left(base):

    print("Moving to the left on the X axis")
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    twist.linear_x = -0.001
    twist.linear_y = -0.0
    twist.linear_z = 0
    twist.angular_x = 0
    twist.angular_y = 0
    twist.angular_z = 0

def Y_Movement_Up(base):

    print("Moving up on the Y axis.")
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    twist.linear_x = 0
    twist.linear_y = 0.001
    twist.linear_z = 0
    twist.angular_x = 0
    twist.angular_y = 0
    twist.angular_z = 0

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True


def Y_Movement_Down(base):

    print("Moving down on the Y axis.")
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    twist.linear_x = 0
    twist.linear_y = -0.001
    twist.linear_z = 0
    twist.angular_x = 0
    twist.angular_y = 0
    twist.angular_z = 0

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True


def Z_Movement_Front(base):

    print("Moving on the z axis going forward.")
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    twist.linear_x = 0
    twist.linear_y = 0.000
    twist.linear_z = 0.001
    twist.angular_x = 0
    twist.angular_y = 0
    twist.angular_z = 0

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True


def Z_Movement_Back(base):

    print("Moving on the z axis going back.")
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    twist.linear_x = 0
    twist.linear_y = 0.000
    twist.linear_z = -0.001
    twist.angular_x = 0
    twist.angular_y = 0
    twist.angular_z = 0

    print ("Sending the twist command for 5 seconds...")
    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(5)

    print ("Stopping the robot...")
    base.Stop()
    time.sleep(1)

    return True





def SendGripperCommands(self, pos):

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        print("Performing gripper test in position...")
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        position = 0.00


        finger.finger_identifier = 1
        
        
        position = pos
        finger.value = position
        print("Going to position {:0.2f}...".format(finger.value))
        self.base.SendGripperCommand(gripper_command)
        time.sleep(1)

class GripperCommandExample:
    def __init__(self, router, proportional_gain = 2.0):

        self.proportional_gain = proportional_gain
        self.router = router

        # Create base client using TCP router
        self.base = BaseClient(self.router)

    def ExampleSendGripperCommands(self):

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        print("Performing gripper test in position...")
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        position = 10
        finger.finger_identifier = 1
        # while position < 1.0:
        #     finger.value = position
        #     print("Going to position {:0.2f}...".format(finger.value))
        #     self.base.SendGripperCommand(gripper_command)
        #     position += 0.1
        #     time.sleep(1)

        # Set speed to open gripper
        print ("Opening gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = 0.1
        self.base.SendGripperCommand(gripper_command)
        gripper_request = Base_pb2.GripperRequest()

        # Wait for reported position to be opened
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len (gripper_measure.finger):
                print("Current position is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value < 0.01:
                    break
            else: # Else, no finger present in answer, end loop
                break

        # Set speed to close gripper
        print ("Closing gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = -0.1
        self.base.SendGripperCommand(gripper_command)

        # Wait for reported speed to be 0
        gripper_request.mode = Base_pb2.GRIPPER_SPEED
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len (gripper_measure.finger):
                print("Current speed is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value == 0.0:
                    break
            else: # Else, no finger present in answer, end loop
                break

# Play Voice 
def playaudio(audio):
    playsound(audio)

def convert_to_audio(text, mp3_value):
    audio = gTTS(text)
    audio.save(mp3_value)
    playaudio(mp3_value)

def basicOBJDetect():
        # Import the utilities helper module
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
        # Utilize for home button.Home position method above.
        success = True
        success &= example_move_to_home_position(base)
        #success &= basicMovement(base, .025, -.05, .03, 0 , 0, 0)

        time.sleep(1.5)

        print("Arm Has returned Home.")

        time.sleep(1.5)


        print("Moving arm to begin scan")
        success &= basicMovement(base, -.025, 0, .0, 0 , 0, 0)

        time.sleep(1.5)

        success &= basicMovement(base, 0, -.057, .0, 0 , 0, 0)

        time.sleep(1.5)

        ##Running Vision

        try:
            # Create a context object. This object owns the handles to all connected realsense devices
            pipeline = rs.pipeline()

            # Configure streams
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

            # Start streaming
            pipeline.start(config)

            #while True:
                # This call waits until a new coherent set of frames is available on a device
                # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
            frames = pipeline.wait_for_frames()
            depth = frames.get_depth_frame()
            #if not depth: continue
            
            #Note to self, throw this in a while loop like you were talking about
            width = depth.get_width()
            height = depth.get_height()
            distance = depth.get_distance (int(width/2), int(height/2))
            print(width)
            print(height)
            print(distance)
            success &= basicMovement(base, .035, 0, .0, 0, 0, 0)

            distance = depth  

                # Print a simple text-based representation of the image, by breaking it into 10x20 pixel regions and approximating the coverage of pixels within one meter
                # coverage = [0]*64
                # for y in range(480):
                #     for x in range(640):
                #         dist = depth.get_distance(x, y)
                #         if 0 < dist and dist < 1:
                #             coverage[x//10] += 1
                    
                #     if y%20 == 19:
                #         line = ""
                #         for c in coverage:
                #             line += " .:nhBXWW"[c//25]
                #         coverage = [0]*64
                #         print(line)
            
            #MOVE FORWARD COMMAND 
            success &= basicMovement(base, 0, 0, .035, 0, 0, 0)

            #GRIPPER COMMAND 
            example = GripperCommandExample(router)
            SendGripperCommands(example, 0.35)

            x = input("Enter X input: ")
            y = input("Enter Y input: ")
            z = input("Enter Z input: ")


            #MOVE BACKWARDS 
            success &= basicMovement(base, 0, 0, -.035, 0, 0, 0)

            #MOVE TO LEFT
            success &= basicMovement(base, -.035, 0, .0, 0, 0, 0)

            #RELEASE GRIPPER
            SendGripperCommands(example, 0.0)

            exit(0)
        #except rs.error as e:
        #    # Method calls agaisnt librealsense objects may throw exceptions of type pylibrs.error
        #    print("pylibrs.error was thrown when calling %s(%s):\n", % (e.get_failed_function(), e.get_failed_args()))
        #    print("    %s\n", e.what())
        #    exit(1)
        except Exception as e:
            print(e)
            pass


        ##end of vision code
       
    

    print("End of program")
    ##return 0 if success else 1


#live camera 
def camera_feed(seconds):
    while True: 
        ret, bgr_frame, depth_frame = rs.get_frame_stream()

        point_x, point_y = 250, 100
        distance_mm = depth_frame[point_y, point_x]

        #cv2.circle(bgr_frame, (point_x, point_y), 8, (0, 0, 255), -1)
        #cv2.putText(bgr_frame, "{} mm".format(distance_mm), (point_x, point_y - 10), 0, 1, (0, 0, 255), 2)


        #cv2.imshow("depth frame", depth_frame)
        cv2.imshow("Robotic Eyesight", bgr_frame)
        

        key = cv2.waitKey(1)
        if key == 27:
            break 

p1 = multiprocessing.Process(target=camera_feed, args=[1])
p2 = multiprocessing.Process(target=basicOBJDetect, args=[1])


##MainMethod
# def main():
#     print("Testing method")
#     camera_feed()
#     basicOBJDetect()


if __name__ == "__main__":
    p1.start()
    p2.start()
    p1.join()
    p2.join()
    #exit(main())

