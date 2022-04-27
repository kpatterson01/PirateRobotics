
#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import time
import sys
import os
import threading

sys.path.insert(0, '../106-Gripper_command/01-gripper_command')

# from kortex_api.TCPTransport import TCPTransport
# from kortex_api.RouterClient import RouterClient
# from kortex_api.SessionManager import SessionManager


# from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

# from kortex_api.autogen.messages import Session_pb2, Base_pb2

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2

from playsound import playsound 
from gtts import gTTS 

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
    # Make sure the arm is in Single Level Servoing mode
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


def main():
    # Import the utilities helper module
    import argparse
    
    convert_to_audio("Hello World my name is Kinova. Would you like me to pour you a drink?", "textaudio.mp3")
    userInput = input("Hello World my name is Kinova. Would you like me to pour you a drink?")
    
    
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
        success = True
        success &= example_move_to_home_position(base)
        success &= basicMovement(base, .025, -.05, .03, 0 , 0, 0)
        convert_to_audio("Okay, great. Place your beverage on the orange post-it note and place your cup on my right side. Type done when finished.", "textaudio1.mp3")

        userInput = input("Okay, great. Place your beverage on the orange post-it note and place your cup on my right side. Type done when finished.")
        example = GripperCommandExample(router)
        SendGripperCommands(example, 0.3)

        print("Moving Object")
        time.sleep(1)
        
        success &= basicMovement(base, 0, .04, .0, 0, 0, 0)

        time.sleep(2)

        print("Pouring a drink")
        convert_to_audio("Pouring your delicious beverage, this is going to be the best drink you'll ever have.", "textaudio2.mp3")

        
        success &= basicMovement(base, 0, .0, .0, 0.02, -10.0, 800)

        print("Stop pouring")

        success &= basicMovement(base, 0, .0, .0, 0.02, 0.1, -19)

        print("Move Object Back to Posted Note")

        success &= basicMovement(base, 0, -.04, .0, 0, 0, 0)

        print("Release Object")
        convert_to_audio("I am done pouring, I hope you enjoy your drink. Do not forget to leave a tip to the PirateRobotics Team. See you next time.", "textaudio3.mp3")
        
        SendGripperCommands(example, 0.0)
       



        print("End of program")
        return 0 if success else 1

if __name__ == "__main__":
    exit(main())
