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

timeoutVal = 30000

class PiezoMotor():
    '''
    Piezo motor class.
    '''

    def __init__(self, serialNumber, channel, pollingRate=250, timeout=10000):
        self.serial_number = serialNumber
        self.polling_rate = pollingRate
        self.connect_piezo_motor()
        self.channel = channel
        self.timeout = timeout

    def connect_piezo_motor(self):
        self.motor = InitiatePiezoMotor(self.serial_number, self.polling_rate)
        self.is_connected = True if not self.motor is None else False

    def move_to_position(self, position):
        '''
        Function for moving the motor to a specified position

        Parameters
        ----------
        position : Float, given in units of mm.
            Target position to move motor to.

        Returns
        -------
        bool
            True if move was successfull otherwise false.

        '''
        try:
            self.motor.MoveTo(self.channel, position, self.timeout)
            return True
        except:
            print('Could not move piezo to target position')
            return False

    def set_timeout(self, timeout):
        '''
        Function for setting the timeout of the piezo motor

        Parameters
        ----------
        timeout : int
            Timeout of motot in ms.

        Returns
        -------
        bool
            True if the timeout was okay and set.

        '''
        if timeout >= 1:
            self.timeout = timeout
            return True
        else:
            print("Timeout NOK")
            return False

    def get_timeout(self):
        '''
        Returns
        -------
        int
            Timeout of piezo motor.

        '''
        return self.timeout

    def move_relative(self, distance):
        '''
        Function for moving the piezo a fixed distance relative to it's
        current position.

        Parameters
        ----------
        distance : Int
            Distance in ticks to move.

        Returns
        -------
        boolean
            True if move was successfull otherwise false.

        '''
        target_position = self.get_position()+distance
        return self.move_to_position(target_position)

    def get_position(self):
        '''
        Returns
        -------
        int
            Current position of piezo(in ticks).

        '''
        try:
            return self.motor.GetPosition(self.channel)
        except:
            print('Could not find piezo position')
            return 0

    def disconnect_piezo(self):
        if self.is_connected:
            self.motor.StopPolling()
            self.motor.Disconnect()
            self.is_connected = False

    def __del__(self):
        self.disconnect_piezo()
        # self.motor.StopPolling()
        # self.motor.Disconnect()


def InitiatePiezoMotor(serialNumber, pollingRate=250):
    '''
    Function for initalizing a piezo motor.

    Parameters
    ----------
    serialNumber : String
        Serialnumber of controller which is being contacted.
    pollingRate : float, optional
        The default is 250. Polling rate of controller.

    Returns
    -------
    motor :
        A PiezoMotor controller object. None if initalization failed.

    '''
    DeviceManagerCLI.BuildDeviceList()
    DeviceManagerCLI.GetDeviceListSize()
    motor = KCubeInertialMotor.CreateKCubeInertialMotor(serialNumber)
    for attempts in range(3):
        try:
            motor.Connect(serialNumber)
        except:
            print("Connection attempt", attempts, "failed")
            if attempts < 2:
                print("Will wait 2 seconds and try again")
                sleep(2)
            else:
                print("Cannot connect to device.\n Please ensure that the \
                      device is connected to your computer and not in use in\
                          any other program!")
                return None
    motor.WaitForSettingsInitialized(5000)
    # configure the stage
    motorSettings = motor.GetInertialMotorConfiguration(serialNumber)
    motorSettings.DeviceSettingsName = 'PIA'
    # update the RealToDeviceUnit converter
    motorSettings.UpdateCurrentConfiguration()
    # push the settings down to the device
    currentDeviceSettings = ThorlabsInertialMotorSettings.GetSettings(motorSettings)

    motor.SetSettings(currentDeviceSettings, True, False)
    # Start polling and enable the device
    motor.StartPolling(pollingRate)
    motor.EnableDevice()

    return motor


class StageMotor():
    '''
    Class for the motors used by the stage. Class currently not in use
    '''
    def __init__(self,serialNumber,pollingRate=200,mmToPixel=16140):
        self.motor = InitiateMotor(serialNumber,pollingRate)
        self.startingPosition = self.motor.GetPosition()
        self.mmToPixel = mmToPixel
    def SetJogSpeed(self,jogSpeed,jogAcc=0.1):
        try:
            self.motor.SetJogVelocityParams(Decimal(jogSpeed),Decimal(jogAcc))
        except:
            print('Could not set jog speed.')
    def __del__(self):
        self.motor.StopPolling()
        self.motor.Disconnect()


def InitiateMotor(serialNumber, pollingRate=250, DeviceSettingsName='Z812'):
    '''
    Function for initalizing contact with a thorlabs k-cube controller object.

    Parameters
    ----------
    serialNumber : String
        Serial number of device to be connected. Written on the back of the
    pollingRate : int, optional
        Polling rate of device in ms. The default is 250.
    DeviceSettingsName : string, optional
        Indicates which type of motor is connectd to the controller.
        The default is 'Z812'.

    Returns
    -------
    motor : k-cube controller
        k-cube controller which can be used to control a thorlabs motor.

    '''
    DeviceManagerCLI.BuildDeviceList()
    DeviceManagerCLI.GetDeviceListSize()

    motor = KCubeDCServo.CreateKCubeDCServo(serialNumber)
    for attempts in range(3):
        try:
            motor.Connect(serialNumber)
        except:
            print("Connection attempt", attempts, "failed")
            if attempts < 2:
                print("Will wait 2 seconds and try again")
                sleep(2)
            else:
                print("Cannot connect to device.\n Please ensure that the \
                      device is connected to your computer and not in use in \
                          any other program!")
                return None
    motor.WaitForSettingsInitialized(5000)
    # configure the stage
    motorSettings = motor.LoadMotorConfiguration(serialNumber)
    motorSettings.DeviceSettingsName = DeviceSettingsName
    # update the RealToDeviceUnit converter
    motorSettings.UpdateCurrentConfiguration()
    # push the settings down to the device
    MotorDeviceSettings = motor.MotorDeviceSettings
    motor.SetSettings(MotorDeviceSettings, True, False)
    # Start polling the device
    motor.StartPolling(pollingRate)

    motor.EnableDevice()
    # Jogging parameters set to minimum
    motor.SetJogVelocityParams(Decimal(0.01), Decimal(0.01))
    return motor


def DisconnectMotor(motor):
    '''
    Function for safely disconnecting a motor so that other programs may use
    it.
    Parameters
    ----------
    motor : Thorlabs motor.
        Motor to be disconnected.

    Returns
    -------
    None.

    '''
    motor.StopPolling()
    motor.Disconnect()


def MoveMotor(motor, distance):
    '''
    Helper function for moving a motor.

    Parameters
    ----------
    motor : thorlabs motor
        Motor to be moved.
    distance : float
        Distance to move the motor.

    Returns
    -------
    bool
        True if the move was a success, otherwise false.

    '''
    if distance > 0.1 or distance < -0.1:
        print("Trying to move too far")
        return False
    # For unknown reason python thinks one first must convert to float but
    # only when running from console...
    motor.SetJogStepSize(Decimal(float(distance)))
    try:
        motor.MoveJog(1, timeoutVal)# Jog in forward direction
    except:
        print( "Trying to move motor to NOK position")
        return False
    return True


def MoveMotorPixels(motor, distance, mmToPixel=16140):
    '''
    Moves motor a specified number of pixels.

    Parameters
    ----------
    motor : TYPE - thorlabs motor
         Motor to be moved
    distance : TYPE number
         Distance to move the motor
    mmToPixel : TYPE number for converting from mm(motor units) to pixels, optional
         The default is 16140, valid for our 100x objective and setup.

    Returns
    -------
    bool
        True if move was successfull, false otherwise.
    '''
    motor.SetJogStepSize(Decimal(float(distance/mmToPixel)))
    try:
        motor.MoveJog(1, timeoutVal)  # Jog in forward direction
    except:
        print( "Trying to move motor to NOK position")
        return False
    return True


def MoveMotorToPixel(motor, targetPixel,
                     currentPixel, maxPixel=1280, mmToPixel=16140):
    '''


    Parameters
    ----------
    motor : TYPE
        DESCRIPTION.
    targetPixel : TYPE
        DESCRIPTION.
    currentPixel : TYPE
        DESCRIPTION.
    maxPixel : TYPE, optional
        DESCRIPTION. The default is 1280.
    mmToPixel : TYPE, optional
        DESCRIPTION. The default is 16140.

    Returns
    -------
    bool
        DESCRIPTION.

    '''
    if(targetPixel<0 or targetPixel>maxPixel): # Fix correct boundries
        print("Target pixel outside of bounds")
        return False
    # There should be a minus here, this is due to the setup
    dx = -(targetPixel-currentPixel)/mmToPixel
    motor.SetJogStepSize(Decimal(float(dx)))
    try:
        motor.MoveJog(1,timeoutVal)# Jog in forward direction
    except:
        print( "Trying to move motor to NOK position")
        return False
    return True

def MoveTrapToPosition(motorX, motorY, targetX, targetY, trapX, trapY):
    '''

    Parameters
    ----------
    motorX : TYPE
        DESCRIPTION.
    motorY : TYPE
        DESCRIPTION.
    targetX : TYPE
        DESCRIPTION.
    targetY : TYPE
        DESCRIPTION.
    trapX : TYPE
        DESCRIPTION.
    trapY : TYPE
        DESCRIPTION.

    Returns
    -------
    TYPE
        DESCRIPTION.

    '''
    x = MoveMotorToPixel(motorX, targetX, trapX) # move X
    y = MoveMotorToPixel(motorY, targetY, trapY) # move Y
    return x and y


def setJogSpeed(motor, jog_speed, jog_acc=0.01):
    """
    Sets the jog-speed in mm/s of the motor as well as the jog acceleration
    """
    return motor.SetJogVelocityParams(Decimal(jog_speed), Decimal(jog_acc))
