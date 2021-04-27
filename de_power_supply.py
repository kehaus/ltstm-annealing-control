"""
Class definition to controla Delta Electonika SM70-22 DC Power supply with 
two analog inputs and one analog output of a Labjack U6. Interface used 
for this is the Analog programmable connection specified in the user manual.


https://www.delta-elektronika.nl/upload/PRODUCT_MANUAL_SM1500_V201808.pdf


date = 05/09/2019
"""

__version__ = "1.1.0"
__author__ = "kha"


# from daqunit import AI_channel ,AO_channel


# ============================================================================
# U3 pin config
# ============================================================================

U3_PIN_CONFIG = { # maps U3 terminal label to internal terminal numbering
    'FIO0':         0,
    'FIO1':         1,
    'FIO2':         2,
    'FIO3':         3,
    'FIO4':         4,
    'FIO5':         5,
    'FIO6':         6,
    'FIO7':         7,
}

U3_DAC_REGISTER = {
    'DAC0':         5000,
    'DAC1':         5002
}

POWER_SUPPLY_WIRING = {
    'V_monitor':    'FIO0',
    'I_monitor':    'FIO1',
    'I_program':    'DAC0',
}


# ============================================================================
# 
# ============================================================================

class DEPowerSupply():
    """ Base class representing Delta Electronica Power supplies.
    
    
	Example:
		>>> d = u3.U3()
		>>> p = DEPowerSupply(d)
		>>> p.get_current()
		>>> p.get_voltage()
		>>> p.set_current(1)


    """
    V_READOUT_MAX 	= 1	# [dummy value]
    V_OUTPUT_MAX 	= 1	# [dummy value]
    I_OUTPUT_MAX	    = 1	# [dummy value]

    
    def __init__(self, d):
        self.u3_pin_config = U3_PIN_CONFIG.copy()
        self.u3_dac_reg = U3_DAC_REGISTER.copy()
        self.pw_wiring = POWER_SUPPLY_WIRING.copy()
        self.d = d
        self._config_FIO_as_analog_input()
        return


    def _config_FIO_as_analog_input(self, num_channels=7):
        """initalizes analog input channels
        
        Number of initialized analog input channels is specified by 
        ``num_channels``. 
        
        """
        FIOEIOAnalog = (2 ** num_channels) - 1
        fios = FIOEIOAnalog & 0xFF
        eios = FIOEIOAnalog // 256
        self.d.configIO(FIOAnalog=fios, EIOAnalog=eios)
        return

    def get_voltage(self):
        """returns output voltage"""
        pin_name = self.pw_wiring['V_monitor']
        volt = self._get_ai_value(pin_name)
        return volt
    
    def get_current(self):
        """returns the ouput current"""
        pin_name = self.pw_wiring['I_monitor']
        volt = self._get_ai_value(pin_name)
        return volt

    def _get_ai_value(self, ai_channel):
        """Reads back value from specified analog input channel
        
        
        Example:
        --------
            >>> d = u3.U3()
            >>> p = DEPowerSupply(d)
            >>> ai_channel = 'FIO0'
            >>> p._get_ai_value(ai_channel)

        """
        volt = self.d.getAIN(
            self.u3_pin_config[ai_channel],
            negChannel=31
        )
        return volt
    
    
    def set_current(self, current):
        """sets  power supply output to specified current"""
        dac_channel = self.pw_wiring['I_program']
        volt = self.convert_to_readout_voltage(current)
        self._set_ao_value(dac_channel, volt)

    def _set_ao_value(self, dac_channel, val):
        """sets the specified DAC channel to the given voltage value ``val``.
        
        Parameter:
        ---------
            dac_channel | 'str'
                specifies the DAC channel to change. Allowed values are 
                ``DAC0`` or ``DAC1``.
            val | float
                value in volt to which the specifed DAC channel is set
        
        Example:
        --------
            >>> d = u3.U3()
            >>> p = DEPowerSupply(d)
            >>> current = 0.5 
            >>> p._set_ao_channel(current)
        
        """
        reg = self.u3_dac_reg[dac_channel]
        self.d.writeRegister(reg, val)
        return 


    @classmethod
    def convert_to_readout_voltage(cls, current, correction_factor=0.1):
        """converts the given current value to the corresponding control-voltage value
        
        Attention
        ---------
            correction factor is necessary because current value entered here does not 
            atual output current on DC power supply.


        """
        return current / cls.I_OUTPUT_MAX * cls.V_READOUT_MAX * (1+correction_factor)


# ============================================================================
# 
# ============================================================================

class ES03010(DEPowerSupply):
    """Represents the ES 300 Series 300Watts DC power supplies


    Example:
        >>> dd = u3.U3()
        >>> es = ES03010(dd)
        >>> es.get_current()
        >>> es.get_voltage()
        >>> es.set_current(1)


    """
    V_READOUT_MAX 	= 5.0	# [V]
    V_OUTPUT_MAX 	= 30	# [V]
    I_OUTPUT_MAX	= 10	# [A]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)



