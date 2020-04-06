from ctypes import *
import clr,sys
from System import Decimal,Int32
clr.AddReference('C:/Program Files/Thorlabs/Kinesis/Thorlabs.MotionControl.DeviceManagerCLI.dll')
clr.AddReference('C:/Program Files/Thorlabs/Kinesis/Thorlabs.MotionControl.GenericMotorCLI.dll')
clr.AddReference('C:/Program Files/Thorlabs/Kinesis/Thorlabs.MotionControl.KCube.DCServoCLI.dll')

from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.DCServoCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import MotorDirection

DeviceManagerCLI.BuildDeviceList()
DeviceManagerCLI.GetDeviceListSize()

# Relevant parameters
mmToPixel = 10000 # 1 mm = 10 000 pixels
timeoutVal = 30000

def InitateMotor(serialNumber,pollingRate=250):
    DeviceManagerCLI.BuildDeviceList()
    DeviceManagerCLI.GetDeviceListSize()

    motor = KCubeDCServo.CreateKCubeDCServo(serialNumber)
    motor.Connect(serialNumber)
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
    #motor.SetJogVelocityParams(Decimal(0.01),Decimal(0.01)) # Jogging parameters set to minimum
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
    motor.MoveJog(1,timeoutVal)# Jog in forward direction
    return True
def MoveMotorToPixel(motor,targetPixel,currentPixel,maxPixel=1280):
    if(targetPixel<0 or targetPixel>maxPixel): # Fix correct boundries
        print("Target pixel outside of bounds")
        return False
    dx = (targetPixel-currentPixel)/mmToPixel
    print(Decimal(float(dx)))
    motor.SetJogStepSize(Decimal(float(dx))) # For unknown reason python thinks one first must convert to float but only when running from console...
    motor.MoveJog(1,timeoutVal)# Jog in forward direction
    return True

def MoveTrapToPosition(motorX,motorY,targetX,targetY,trapX,trapY):
    """
    Moves the trap to target position
    """
    x=MoveMotorToPixel(motorX,targetX,trapX) # move X
    y=MoveMotorToPixel(motorY,targetY,trapY) # move Y
    return x and y

# print("Forwards = ",MotorDirection.Forward)
# print("Backwards = ",MotorDirection.Backward)
"""

serialNumX='27502438'
serialNumY='27502419'
pollingRate=50
timeout_val=60000


motor = InitateMotor(serialNumX,pollingRate=pollingRate)

print(motor.Position)
targetPosX = Decimal(0.01)
timeoutVal = 10000
motor.MoveTo(targetPosX, timeoutVal)

DisconnectMotor(motor)

"""




# motor = KCubeDCServo.CreateKCubeDCServo(serialNumX)
# motor.Connect(serialNumX)
# motor.WaitForSettingsInitialized(5000)
#
# # configure the stage
# motorSettings = motor.LoadMotorConfiguration(serialNumX)
# motorSettings.DeviceSettingsName = 'Z812'
#
# # update the RealToDeviceUnit converter
# motorSettings.UpdateCurrentConfiguration()
#
# # push the settings down to the device
# MotorDeviceSettings = motor.MotorDeviceSettings
# motor.SetSettings(MotorDeviceSettings, True, False)
# # Start polling the device
# motor.StartPolling(pollingRate)
#
# motor.EnableDevice() # IS this needed?
