# Control of the temperature controller

import pyvisa
def get_resource_list():
    """
    Returns a list of available pyvisa resources
    """
    rm = pyvisa.ResourceManager()
    return rm.list_resources()

class TED4015():
    """
    Class for controlling the TED4015 instrument.
    Initiates the instrument to upon creation. Use with care since error handling is
    essentially not present
    """
    def __init__(self,index=0):
        self.rm = pyvisa.ResourceManager()
        self.resource_list = self.rm.list_resources()
        self.TED4015 = self.rm.open_resource(self.resource_list[index])
        self.TED4015.read_termination = '\n'
        self.TED4015.write_termination = '\n'
    def get_setpoint_temperature(self):
        """
        Returns the setpoint temperature of the instrument
        """
        return float(self.TED4015.query('SOUR:TEMP?'))
    def set_setpoint_temperature(self,temperature):
        """
        Sets the target temperature(setpoint temperature) of the instrument
        """
        # TODO are there any limits to te temperature?
        temperature_command = 'SOUR:TEMP '+str(temperature)+'C'
        return self.TED4015.write(temperature_command)
    def query_output(self):
        """
        Checks if the output is on or off.
        0 => off
        1 => on
        """
        return self.TED4015.query('OUTP?')
    def turn_on_output(self):
        """
        Activates the output of the TED4015
        """
        return self.TED4015.write('OUTP ON')
    def turn_off_output(self):
        """
        Deactivates the output of the TED4015
        """
        return self.TED4015.write('OUTP OFF')
    def measure_temperature(self):
        """
        Measueres the temperature of the thermocouple.
        Note that this is not the same as the setpoint temperature.
        """
        return float(self.TED4015.query('MEAS:SCAL:TEMP?'))
    def measure_power(self):
        """
        Returns the instananeous power consumption of the intrument
        """
        return float(self.TED4015.query('MEAS:SCAL:POW?'))
    def close_device(self):
        """
        Closes the communication to the device
        """
        return self.TED4015.close()
