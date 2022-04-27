## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#####################################################
## librealsense tutorial #1 - Accessing depth data ##
#####################################################

# First import the library
import pyrealsense2 as rs


import time
import sys
import os
import threading
#import IT3ItemPickUp.py

sys.path.insert(0, '../106-Gripper_command/01-gripper_command')


from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2

from playsound import playsound 
from gtts import gTTS 

class GripperLowLevelExample:
    def __init__(self, router, router_real_time, proportional_gain = 2.0):
        """
            GripperLowLevelExample class constructor.

            Inputs:
                kortex_api.RouterClient router:            TCP router
                kortex_api.RouterClient router_real_time:  Real-time UDP router
                float proportional_gain: Proportional gain used in control loop (default value is 2.0)

            Outputs:
                None
            Notes:
                - Actuators and gripper initial position are retrieved to set initial positions
                - Actuator and gripper cyclic command objects are created in constructor. Their
                  references are used to update position and speed.
        """

        self.proportional_gain = proportional_gain

        ###########################################################################################
        # UDP and TCP sessions are used in this example.
        # TCP is used to perform the change of servoing mode
        # UDP is used for cyclic commands.
        #
        # 2 sessions have to be created: 1 for TCP and 1 for UDP
        ###########################################################################################

        self.router = router
        self.router_real_time = router_real_time

        # Create base client using TCP router
        self.base = BaseClient(self.router)

        # Create base cyclic client using UDP router.
        self.base_cyclic = BaseCyclicClient(self.router_real_time)

        # Create base cyclic command object.
        self.base_command = BaseCyclic_pb2.Command()
        self.base_command.frame_id = 0
        self.base_command.interconnect.command_id.identifier = 0
        self.base_command.interconnect.gripper_command.command_id.identifier = 0

        # Add motor command to interconnect's cyclic
        self.motorcmd = self.base_command.interconnect.gripper_command.motor_cmd.add()

        # Set gripper's initial position velocity and force
        base_feedback = self.base_cyclic.RefreshFeedback()
        self.motorcmd.position = base_feedback.interconnect.gripper_feedback.motor[0].position
        self.motorcmd.velocity = 0
        self.motorcmd.force = 100

        for actuator in base_feedback.actuators:
            self.actuator_command = self.base_command.actuators.add()
            self.actuator_command.position = actuator.position
            self.actuator_command.velocity = 0.0
            self.actuator_command.torque_joint = 0.0
            self.actuator_command.command_id = 0
            print("Position = ", actuator.position)

        # Save servoing mode before changing it
        self.previous_servoing_mode = self.base.GetServoingMode()

        # Set base in low level servoing mode
        servoing_mode_info = Base_pb2.ServoingModeInformation()
        servoing_mode_info.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
        self.base.SetServoingMode(servoing_mode_info)

    def Cleanup(self):
        """
            Restore arm's servoing mode to the one that
            was effective before running the example.

            Inputs:
                None
            Outputs:
                None
            Notes:
                None

        """
        # Restore servoing mode to the one that was in use before running the example
        self.base.SetServoingMode(self.previous_servoing_mode)


    def Goto(self, target_position):
        """
            Position gripper to a requested target position using a simple
            proportional feedback loop which changes speed according to error
            between target position and current gripper position

            Inputs:
                float target_position: position (0% - 100%) to send gripper to.
            Outputs:
                Returns True if gripper was positionned successfully, returns False
                otherwise.
            Notes:
                - This function blocks until position is reached.
                - If target position exceeds 100.0, its value is changed to 100.0.
                - If target position is below 0.0, its value is set to 0.0.
        """
        if target_position > 100.0:
            target_position = 100.0
        if target_position < 0.0:
            target_position = 0.0
        while True:
            try:
                base_feedback = self.base_cyclic.Refresh(self.base_command)

                # Calculate speed according to position error (target position VS current position)
                position_error = target_position - base_feedback.interconnect.gripper_feedback.motor[0].position

                # If positional error is small, stop gripper
                if abs(position_error) < 1.5:
                    position_error = 0
                    self.motorcmd.velocity = 0
                    self.base_cyclic.Refresh(self.base_command)
                    return True
                else:
                    self.motorcmd.velocity = self.proportional_gain * abs(position_error)
                    if self.motorcmd.velocity > 100.0:
                        self.motorcmd.velocity = 100.0
                    self.motorcmd.position = target_position

            except Exception as e:
                print("Error in refresh: " + str(e))
                return False
            time.sleep(0.001)
        return True

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

def basicMovement(base, linX, linY, linZ, angX, angY, angZ,):

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
    time.sleep(2)

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




##MainMethod

def main():
    # Import the utilities helper module
    import argparse 
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities
    # Parse arguments
    args = utilities.parseConnectionArguments()

    itemsInBox = 0
    
    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        with utilities.DeviceConnection.createUdpConnection(args) as router_real_time:
                # Create required services
                base = BaseClient(router)
                # Example core
                print("Returning Arm to Home")

                # Utilize for home button.Home position method above.
                success = True
                success &= example_move_to_home_position(base)

                #giving user option to select what Item they want
                #divided by section on the shelf (for example, section 1 would contain water bottles, section 2 ramen, etc.)

                print("Enter the item you would like to grab. ")
                userSelection = input("Enter W for water bottle, R for ramen, J for juice, S for Sparkling Ice, or D for Dots. ")
                if userSelection == "W":
                    print("You have selected water")

                if userSelection == "R":
                    print("You have selected ramen")

                if userSelection == "J":
                    print("You have selected juice")

                if userSelection == "S":
                    print("You have selected sparkling Ice")

                if userSelection == "D":
                    print("You have selected dots")

                print("Moving to that section of the shelf")

                success &= basicMovement(base, .09, 0, 0, 0 , 8, 0)

                success &= basicMovement(base, .04, 0, 0, 0 , 11, 0)
                success &= basicMovement(base, .04, -.03,-.003, 0 , 0, 0)
                success &= basicMovement(base, 0, 0, 0, 0 , 0, 0)
                #success &= basicMovement(base, 0, 0, 0, 0, -5, 0)
                #Y VALUE FOR LASAT MOVEMENT WAS -.03


                if userSelection == "W":
                    success &= basicMovement(base, 0, 0, 0, 0 , 0, 0)

                if userSelection == "R":
                    success &= basicMovement(base, -.03, 0, 0, 0 , 0, 0)

                if userSelection == "J":
                    success &= basicMovement(base, -.06, 0, 0, 0 , 0, 0)

                if userSelection == "S":
                    success &= basicMovement(base, -.08, 0, 0, 0 , 0, 0)

                if userSelection == "D":
                    success &= basicMovement(base, -.10, 0, 0, 0 , 0, 0)






                #print("Welcome to our grocery store, please select the item you would like to place in your box:")
                print("Our arm will now use the camera to scan for groceries and place them into the box, 1 by 1.")

                #try:
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
                
                #if not depth: continue
                
                #Note to self, throw this in a while loop like you were talking about
                #Finding the minimum distance of object on platform. 
                xMove = 0


                # print ("Starting distance and minimum distance ")
                # print(distance)
                # print(miniDistance)

                #Initializing values to prevent them default to 0

                # frames = pipeline.wait_for_frames()
                # depth = frames.get_depth_frame()
                # width = depth.get_width()
                # height = depth.get_height()                
                # distance = depth.get_distance (int(width/2), int(height/2))

                # frames = pipeline.wait_for_frames()
                # depth = frames.get_depth_frame()
                # width = depth.get_width()
                # height = depth.get_height()                
                # distance = depth.get_distance (int(width/2), int(height/2))

                miniDistance = 100
                distance = 100
                
                while (xMove < .050):
                    
                    success &= basicMovement(base, -.0075, 0, .0, 0, 0, 0)
                    xMove = xMove + .005

                    frames = pipeline.wait_for_frames()
                    depth = frames.get_depth_frame()
                    width = depth.get_width()
                    height = depth.get_height()
                    distance = depth.get_distance (int(width/2), int(height/2))
                    
                    print("Depth ="+str(depth))
                    #print("Width"+str(width))
                    # print("Height"+str(height))
                    print("Current Distance"+str(distance))
                    #print("xMove"+str(xMove))
                    print("Minimum distance found"+str(miniDistance))
                    print(miniDistance)


                    if distance < miniDistance:
                        print("New Minimum Distance Found")
                        miniDistance = distance
                        
                    if miniDistance == 0:
                        print("Object Detected")
                        break
                        
                print("Done Scanning")

                print("Determining distance from object")
                frames = pipeline.wait_for_frames()
                depth = frames.get_depth_frame()
                width = depth.get_width()
                height = depth.get_height()                
                distance = depth.get_distance (int(width/2), int(height/2))

                print("Width:"+str(width))
                print("height:"+str(height))
                print("Depth:"+str(depth))
                print("Distance:"+str(distance))

                example = GripperCommandExample(router)
                SendGripperCommands(example, 0.0)  

                print("Grabbing object")
            
                success &= basicMovement(base, 0, .01, 0, 0 , 0, 0)
                success &= basicMovement(base, 0, 0, .03, 0 , 0, 0)



                # # #GRIPPER COMMAND
                if userSelection == "J":
                    example = GripperCommandExample(router)
                    SendGripperCommands(example, .55)

                else:
                    example = GripperCommandExample(router)
                    SendGripperCommands(example, .65)

                print("Item grabbed, bringing item back to home ")

                success &= basicMovement(base, 0, 0.04, 0, 0 , 0, 0)
                example_move_to_home_position(base)

                print("Arm Has returned Home.")





                    
                    #exit(1)
                # except rs.error as e:
                # #    # Method calls agaisnt librealsense objects may throw exceptions of type pylibrs.error
                # #    print("pylibrs.error was thrown when calling %s(%s):\n", % (e.get_failed_function(), e.get_failed_args()))
                # #    print("    %s\n", e.what())
                #     exit(1)
                #except Exception as e:
                    #print(e)
                    #pass


                ##end of vision code
                print("End of program")



if __name__ == "__main__":
    exit(main())