from ctypes import *
import clr,sys
from System import Decimal,Int32
from time import sleep
"""
Note when usin this code on other computer than the one in the biophysics lab these paths may need changing.
"""
clr.AddReference('C:/Program Files/Thorlabs/Kinesis/Thorlabs.MotionControl.DeviceManagerCLI.dll')
clr.AddReference('C:/Program Files/Thorlabs/Kinesis/Thorlabs.MotionControl.GenericMotorCLI.dll')
clr.AddReference('C:/Program Files/Thorlabs/Kinesis/Thorlabs.MotionControl.KCube.DCServoCLI.dll')
clr.AddReference('C:/Program Files/Thorlabs/Kinesis/Thorlabs.MotionControl.KCube.InertialMotorCLI.dll ')

from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.DCServoCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import MotorDirection
from Thorlabs.MotionControl.KCube.InertialMotorCLI import *

DeviceManagerCLI.BuildDeviceList()
DeviceManagerCLI.GetDeviceListSize()

# Relevant parameters
# mmToPixel = 10000/0.6 # 1 mm = 10 000 pixels for 60x objective # Double check the value

# TODO change this to a class?
timeoutVal = 30000


def InitatePiezoMotor(serialNumber,pollingRate=250):
    DeviceManagerCLI.BuildDeviceList()
    DeviceManagerCLI.GetDeviceListSize()

    motor = KCubeInertialMotor.CreateKCubeInertialMotor(serialNumber)
    for attempts in range(3):
        try:
            motor.Connect(serialNumber)
        except:
            print("Connection attempt",attempts,"failed")
            if(attempts<2):
                print("Will wait 5 seconds and try again")
                sleep(5)
            else:
                print("Cannot connect to device.\n Please ensure that the device is connected to your computer and not in use in any other program!")
    motor.WaitForSettingsInitialized(5000)
    deviceInfo = motor.GetDeviceInfo()
    print(deviceInfo,'\n')
    # configure the stage
    motorSettings = motor.GetInertialMotorConfiguration(serialNumber)
    motorSettings.DeviceSettingsName = 'PIA'
    # update the RealToDeviceUnit converter
    motorSettings.UpdateCurrentConfiguration()
    # push the settings down to the device
    #MotorDeviceSettings = motor.MotorDeviceSettings
    currentDeviceSettings = ThorlabsInertialMotorSettings.GetSettings(motorSettings);
    # TODO specify channel 1
    motor.SetSettings(currentDeviceSettings, True, False) # What does tru/false mean here?
    # Start polling the device
    motor.StartPolling(pollingRate)

    motor.EnableDevice()
    #motor.SetJogVelocityParams(Decimal(0.01),Decimal(0.01)) # Jogging parameters set to minimum
    return motor

def InitateMotor(serialNumber,pollingRate=250):
    DeviceManagerCLI.BuildDeviceList()
    DeviceManagerCLI.GetDeviceListSize()

    motor = KCubeDCServo.CreateKCubeDCServo(serialNumber)
    for attempts in range(3):
        try:
            motor.Connect(serialNumber)
        except:
            print("Connection attempt",attempts,"failed")
            if(attempts<2):
                print("Will wait 5 seconds and try again")
                sleep(5)
            else:
                print("Cannot connect to device.\n Please ensure that the device is connected to your computer and not in use in any other program!")
    motor.WaitForSettingsInitialized(5000)
    # configure the stage
    motorSettings = motor.LoadMotorConfiguration(serialNumber)
    motorSettings.DeviceSettingsName = 'Z812'
    # update the RealToDeviceUnit converter
    motorSettings.UpdateCurrentConfiguration()
    # push the settings down to the device
    MotorDeviceSettings = motor.MotorDeviceSettings
    motor.SetSettings(MotorDeviceSettings, True, False)
    # Start polling the device
    motor.StartPolling(pollingRate)

    motor.EnableDevice()
    motor.SetJogVelocityParams(Decimal(0.01),Decimal(0.01)) # Jogging parameters set to minimum
    return motor
def DisconnectMotor(motor):
    motor.StopPolling()
    motor.Disconnect()
def MoveMotor(motor,distance):
    # Jog motor one step along distance
    if distance>0.1 or distance<-0.1:
        print("Trying to move too far")
        return False
    motor.SetJogStepSize(Decimal(float(distance))) # For unknown reason python thinks one first must convert to float but only when running from console...
    try:
        motor.MoveJog(1,timeoutVal)# Jog in forward direction
    except:
        print( "Trying to move motor to NOK position")
        return False
    return True
def MoveMotorToPixel(motor,targetPixel,currentPixel,maxPixel=1280,mmToPixel=16140): # Double check mm to pixel value empirically measured
    if(targetPixel<0 or targetPixel>maxPixel): # Fix correct boundries
        print("Target pixel outside of bounds")
        return False
    dx = -(targetPixel-currentPixel)/mmToPixel # For some reason there should be a minus here, suspec this is due to changes in the setup
    motor.SetJogStepSize(Decimal(float(dx))) # For unknown reason python thinks one first must convert to float but only when running from console...
    try:
        motor.MoveJog(1,timeoutVal)# Jog in forward direction
    except:
        print( "Trying to move motor to NOK position")
        return False
    return True

def MoveTrapToPosition(motorX,motorY,targetX,targetY,trapX,trapY):
    """
    Moves the trap to target position
    """
    x=MoveMotorToPixel(motorX,targetX,trapX) # move X
    y=MoveMotorToPixel(motorY,targetY,trapY) # move Y
    return x and y
def setJogSpeed(motor,jog_speed,jog_acc=0.01):
    """
    Sets the jog-speed in mm/s of the motor as well as the jog acceleration
    """
    return motor.SetJogVelocityParams(Decimal(jog_speed),Decimal(jog_acc))
