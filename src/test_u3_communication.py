#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
"""

import os
import u3


# ============================================================================
# U3 Simple test communications
# 
# Examples from here:
#       * https://labjack.com/support/datasheets/u3/hardware-description/ain/channel_numbers
#       * https://github.com/labjack/LabJackPython/blob/master/Examples/u3allio.py
#
#
#
# DAC0: register 5000
# DAC1: register 5001
#    from link: https://github.com/labjack/LabJackPython/blob/master/src/u3.py
# ============================================================================


# ===========
# initialized the Labjack U3
# ===========
d = u3.U3()


# ===========
# Configure Analog Input channels
# ===========
num_channels = 2

FIOEIOAnalog = (2 ** num_channels) - 1
fios = FIOEIOAnalog & 0xFF
eios = FIOEIOAnalog // 256
d.configIO(FIOAnalog=fios, EIOAnalog=eios)



# ===========
# change voltage output at DAC0
# ===========
volt = 1.00 # [V]
d.writeRegister(5000, volt)



# ===========
# change voltage output at DAC1
# ===========
volt = 0.75 # [V]
d.writeRegister(5002, volt)


# ===========
# Read out analog input channel FIO0
# ===========
volt = d.getAIN(0, 31)

# ===========
# Read out analog input channel FIO1
# ===========
volt = d.getAIN(1, 31)










