#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script contains class representation of the Ferrovac STM stepper piezo 
controller. 

This class relies on using a U3 to send digital signal to the stepper motor
controller



"""

import time

import numpy as np
import u3


# ============================================================================
# pin configuration
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
    'GND':          None,
}

U3_WIRING = { # maps DSUB9 cable color to U3 terminal label; this needs to correspond to how it is plugged in.
    'brown':        'FIO0',
    'red':          'FIO1',
    'orange':       'FIO2',
    'blue':         'FIO5',
    'violet':       'FIO6',
    'green':        'FIO7',
    'black':        'GND',
    'yellow':       'n.c.'
}

DSUB9_PIN_CONFIG = { # maps DSUB9 (+ converter plug) pin number to  DSUB9 cable color
    1:              'black',
    2:              'brown',
    3:              'red',
    4:              'orange',
    5:              'n.c.',
    6:              'yellow',
    7:              'blue',
    8:              'violet',
    9:              'green'
}

CONTROLLER_PIN_CONFIG = { # maps controller function to DSUB9-output-plug pin number
    'GND':          1,
    'bit0':         2,
    'bit1':         3,
    'bit2':         4,
#    '5V':           6,
    'LED_burst':    7,
    'LED_single':   8,
    'LED_cont':     9,
}


def compile_wiring_dict():
    """returns dct that maps keys from CONTROLLER_PIN_CONFIG dct to values from 
    U3_PIN_CONFIG dct using the intermediary dictionaries
    
    The dicionaries used here resemble the physical connects between the 
    different components involved (e.g pin-to-wire-color: wire-color to U3 Pin 
    label: U3 pin label to internal pin assignement). This function is then 
    used to establish a direct mapping between the keys and values of the first
    and last dictionary of this chain. 
    
    Attention:
    ----------
    The wiring, as it is specified in the dictionaries above, resembles the
    physical wiring. Therefore it is specific for every setup and needs to be 
    adjusted if the setup changes.
    
    Returns:
    --------
    dct
        maps Ferrovac piezo controller output pins to U3 channel numbers
        
    """    
    pin_dct = compress_dict(
        CONTROLLER_PIN_CONFIG,
        DSUB9_PIN_CONFIG,
        U3_WIRING,
        U3_PIN_CONFIG
    )
    return pin_dct
    

def compress_dict(*args):
    """returns dictionary which maps keys of first dict given to values of last
    dict provided.
    
    This function takes a multiple dictionaries as input argument. It constructs
    a dictionary consisting of the keys of the first dict mapped to the values 
    of the last dictionary provided by using all the mappings between the dicts
    between first and last. This function requires that the values of a dict
    are present as keys of the following dict to be able to establish a mapping.
    
    Return:
    -------
    dct
        dictionary mapping keys of first dict to values of last dct
    
    Example:
    --------
        >>> d1 = {'1':'A1', '2':'A2', '3': 'A3'}
        >>> d2 = {'A1': 'B1', 'A2':'B2', 'A3':'B3'}
        >>> d3 = {'B1':'C1', 'B2':'C2', 'B3':'C3'}
        >>> compress_dict(d1, d2, d3)
        ... {'1': 'C1', '2': 'C2', '3': 'C3'}
    
    Raises:
    -----------
    ``ValueError`` 
        if input arguments are not of type dict
    ``KeyError`` 
        if values and keys of subsequent dicts do not match
    
    """
    for arg in args:
        if type(arg) != dict:
            raise ValueError(
                'input argument {} not valid. Must be of type dict'.format(arg)
            )
    dct_lst = list(args)
    
    dct = dct_lst.pop(0)
    while len(dct_lst) > 0:
        tmp_ = dct_lst.pop(0)
        if not all(val in tmp_.keys() for val in dct.values()):
            raise KeyError(
                'Values and keys of subsequent dictionaries are not consistent.'
            )
        dct = {k: tmp_[v] for (k,v) in dct.items()}
    
    return dct

# ===========================================================================
# stepper motor abstraction class
# ===========================================================================
MOTOR_DIRECTIONS = {
    'left':     [1, 0, 1],
    'right':    [1, 1, 0],
    'back':     [0, 1, 0],
    'down':     [1, 0, 0],
    'fwd':      [0, 0, 1],
    'up':       [0, 1, 1],
    'stop':     [1, 1, 1],
}

class StepperMotorException(Exception):
    """ """

class StepperMotorControl():
    """This class allows to send commands to the Ferrovac STM piezo controller.
    
    This class uses a the digital input and output pins of a LabJack U3 to send
    logic pulses to the Ferrovac STM piezo controller to trigger coarse motion
    movements. This class also allows to read out the piezo controller movement
    status (e.g. continous, burst, single).
    
    Requirements:
    -------------
    * U3 is connected to computer and labjack-python library is installed
    * U3 dio channels are connected to the Ferrovac STM controller DSUB9 
    output pin
    * correct wiring schematics is indicated in the python dictionaries
    `` U3_WIRING``and ``CONTROLLER_PIN_CONFIG`
    
    Example:
    --------
    >>> sm = StepperMotorControl()
    >>> sm.get_motor_setting()  # returns motor settings
    ... continous               # burst, single, auto are also possible responses
    >>> sm.walk('down', 1)      # walks one second downwards
    
    
    """
    
    MOVEMENT_SETTINGS = [
        'LED_cont', 'LED_burst', 'LED_single'
    ]
    MOTOR_DIRECTIONS = {
        'left':     [1, 0, 1],
        'right':    [1, 1, 0],
        'back':     [0, 1, 0],
        'down':     [1, 0, 0],
        'fwd':      [0, 0, 1],
        'up':       [0, 1, 1],
        'stop':     [1, 1, 1],
    }
    BIT_LIST = [
        'bit0', 'bit1', 'bit2'
    ]
    BURST_PULSE_DURATION = 0.15
    
    def __init__(self):

#        self.pin_dct = pin_dct
        self.pin_dct = compile_wiring_dict()
        self.mot_dir = self._compile_motor_directions_dict()
        
        self._init_u3()
        
        
    def _init_u3(self):
        """initializes the U3. 
        
        This function requires the U3 being connceted to
        the computer.
        """
        self.d = u3.U3()
        self.d.configU3()
        
        
        
    def _compile_motor_directions_dict(self):
        """returns dct which maps human-readable movement-direction names and
        its bit pattern to the correct pins of the piezo controller.
        
        This function uses the mapping of movement direction and bit pattern
        defined in ''MOTOR_DIRECTIONS'' to associate it with the pins from the 
        piezo controller DSUB9 output connector.
        
        Return:
        -------
        dct 
            mapping of movement direction and bit pattern to piezo controller 
            pin configuration
        
        """
        bit_lst = StepperMotorControl.BIT_LIST.copy()
        
        dct = {
            key: {
                k:v for (k,v) in zip(bit_lst, val)
            } for (key, val) in StepperMotorControl.MOTOR_DIRECTIONS.items()
        }
        return dct
    
    def _check_motor_direction(self, direction):
        """check if given direction is valid. Raises StepperMotorException 
        otherwise
        """
        msg = "Passed direction {0} is not valid. Try instead: {1}".format(
                direction,
                self.mot_dir.keys()
        )
        
        if not direction in self.mot_dir.keys():
            raise StepperMotorException(msg)
            
    def _convert_n_steps_to_int(self, n_steps):
        """returns n_steps as integer. Raises ``StepperMotorException`` if 
        conversion is not possible
        
        Return :
        --------
        int
            number of steps to move
        
        """
        try:
            n_steps = int(n_steps)
        except ValueError:
            msg = "given n_steps value {} is not a valid step number".format(
                str(n_steps)
            )
            raise StepperMotorException(msg)            
        return n_steps

    def _get_dio_state(self, io_num):
        return self.d.getDIOState(io_num)

    def get_motor_setting(self):
        """returns the motion setting of the piezo controller
        
        It defines the type of motion of the piezo motors. Possible settings 
        are:
            * continous, 
            * burst
            * single
            * auto
            
        This setting is read from the controller. It can only manualy be 
        changed at the controller and not through this python interface.
        
        Return :
        --------
        str
            piezo controller movement settings
         
            
        """
        movement_setting = StepperMotorControl.MOVEMENT_SETTINGS.copy()
        
        setting = 'auto'
        for s in movement_setting:
            io_num = self.pin_dct[s]
            state = self._get_dio_state(io_num)
            
            if state == 0:
                setting = s.replace('LED_','')
        
        return setting
    

    def _walk(self, direction):
        """changes the controller movement direction 
        
        controller movement direction is changed by setting the bit pattern at 
        corresponding U3 digital output pins. This function only changes 
        movement direction. To generate pulses (e.g. movement-direction-change
        sequence of STOP-RIGHT-STOP) you have to use the function ``self.walk``
        
        For the specifics of how to compose command lists and send them to the
        Labjack U3 (as it is implemented here) check out the Labjack 
        documentation.
        
        Parameter: 
        ----------
        direction | str
            specifies the movement direction. 
            
        """
        self._check_motor_direction(direction)
        
        bit_pattern = self.mot_dir[direction]
        command_lst = []
        for bit, state in bit_pattern.items():
            io_num = self.pin_dct[bit]
            
            command_lst.append(
                u3.BitStateWrite(io_num, state)
            )
        self.d.getFeedback(*command_lst)        
        return  

    def walk(self, direction, duration=0.1):
        """walks in given direction for specified duration
        
        Functions sends a movement trigger pulse to the controller consisting
        of a sequence STOP-DIRECTION-STOP. Depending on what the current 
        controller movement settings are triggers this pulse different 
        responses-. E.g. if movement setting is set to *continuous* the pulse 
        duration specifies how long the piezo controller moves in the specified 
        direction; If the movement setting is *single" the pulse will only 
        trigger one step no matter how long the pulse is.
        
        Parameter:
        ----------
        direction | str
            direction along which controller moves
        duration | float
            time duration of movement in seconds
            
        
        """
        self._walk('stop')
        self._walk(direction)
        time.sleep(duration*0.5)
        self._walk('stop')
        time.sleep(duration*0.5)

    def walk_steps(self, direction, n_steps, duration_per_step = 0.01):
        """walks given number of steps in specified direction.
        
        Function sends sequence of pulses to controller to trigger a specifed
        number of single-step motions. The keyword argument allows to tune the 
        pulse duration. This function returns a ``StepperMotorException`` if 
        the controller movement is not set to *single*. Since function operates 
        in single-step mode, this function does not influence moved distance as 
        long as it is long enough to that controller can react to all trigger 
        pulses.
        
        Parameter:
        ----------
        direction | str
            direction along controller moves
        n_steps | int
            number of steps moved by controller
        duration_per_step | float
            specifies pulse duration in seconds. 
        
        """
        n_steps = self._convert_n_steps_to_int(n_steps)
        
        setting = self.get_motor_setting()
        err_msg = """Need to switch motor control setting to single. """
        if setting != 'single':
            raise StepperMotorException(err_msg)
        
        for i in range(n_steps):
            self.walk(direction, duration=duration_per_step)
            time.sleep(0.01)
        return
    
    def walk_bursts(self, direction, n_steps):
        """walkt given number of bursts in specified direction

        Function sends sequence of pulses to controller to trigger a specifed
        number of single-burst motions. The keyword argument allows to tune the 
        pulse duration. This function returns a ``StepperMotorException`` if 
        the controller movement is not set to *burst*. Since function operates 
        in single-burst mode, this function does not influence moved distance as 
        long as it is long enough to that controller can react to all trigger 
        pulses. A burst step requires a defined pulse duration. Therefore the 
        burste pulse duration is no a accessible parameter in this function.
        
        Parameter:
        ----------
        direction | str
            direction along controller moves
        n_steps | int
            number of steps moved by controller
        
        """
        n_steps = self._convert_n_steps_to_int(n_steps)
        
#        if duration_per_burst is None:
#            duration_per_burst = self.BURST_PULSE_DURATION
        
        setting = self.get_motor_setting()
        err_msg = """Need to switch motor control setting to burst. """
        if setting != 'burst':
            raise StepperMotorException(err_msg)
            
        for i in range(n_steps):
            self.walk(direction, duration=self.BURST_PULSE_DURATION)
            time.sleep(0.01)
        return        
            
    def _calibrate_motion_axes(self, direction, n_steps):
        """ 
        
        **incomplete**
        
        Example:
        --------
        'right', 50000
        """
        
        CALIB_MAT = np.array([
            [1., 0., 0.],
            [0., 0., 0.],
            [-0.2, 0., 0.]
        ])
        
    def _convert_motion_to_cartesian(direction, n_steps):
        """ """
        conv_table = {
            'fwd':          np.array([[ 1],[ 0],[ 0]]),
            'back':         np.array([[-1],[ 0],[ 0]]),
            'left':         np.array([[ 0],[ 1],[ 0]]),
            'right':        np.array([[ 0],[-1],[ 0]]),
            'down':         np.array([[ 0],[ 0],[ 1]]),
            'up':           np.array([[ 0],[ 0],[-1]]),
        }
        return conv_table[direction]*n_steps
    
    
    def _convert_cartesian_to_motion(cart_vec):
        """ """
        
        mov_x = {1: 'fwd', -1: 'back'}
        mov_y = {1: 'left', -1: 'right'}
        mov_z = {1: 'down', -1: 'up'}
        conv_table = [mov_x, mov_y, mov_z]
        move_sequ = []
        
        for idx, val_arr in enumerate(cart_vec):
            val = val_arr.item()
            sign = np.sign(val, dtype=np.int)
            if sign != 0:
                move_sequ.append(
                    (conv_table[idx][sign], val*sign)
                )
        return move_sequ
        
        
        
# ===========================================================================
# Advanced stepper motor controller
# ===========================================================================
TURN_AROUND_STEP_SEQUENCE = {
    'fwd_to_back':      [('fwd', 100), ('back', 55)],   # back to 56?
    'back_to_fwd':      [('back', 50), ('fwd', 110)]  #50,48,46, 54; fwd:100
}



RATIO_RIGHT_TO_LEFT = 87./78.
#RATIO_BACK_TO_FWD = 470./205.
#RATIO_BACK_TO_FWD = 425./180.
RATIO_BACK_TO_FWD = 450./205.


CONV_TO = {
    'left':         RATIO_RIGHT_TO_LEFT,
    'right':        1./RATIO_RIGHT_TO_LEFT,
    'fwd':          RATIO_BACK_TO_FWD,
    'back':         1./RATIO_RIGHT_TO_LEFT
}

# ===========================
# OPPOSIE_DIR directory helps to easily get the opposite direction 
#    (e.g. OPPOSITE_DIR['left'] -> 'right')
# 
# ===========================
OPPOSITE_DIR = {
    'fwd':      'back',
    'back':     'fwd',
    'right':    'left',
    'left':     'right',
    'up':       'down',
    'down':     'up'
}

# ===========================
# TURN_AROUND_FROM directory helps to easily the turn-around sequence name
#     from providing starting direction 
#    (e.g. TURN_AROUND_FROM['left'] -> 'left_to_right')
# 
# ===========================

TURN_AROUND_FROM = {
    'left':     'left_to_right',
    'right':    'right_to_left',
    'fwd':      'fwd_to_back',
    'back':     'back_to_fwd'
}


class AdvancedStepperMotorException(Exception):
    pass


class AdvancedStepperMotorControl(StepperMotorControl):
    """Provides advanced control features for the ferrovac STM stepper motor
    
    This class is built ontop of the StepperMotorControl class. It implements
    advanced control features like hysteresis correction at 180-degree direction
    changes (i.e. ``turn_around()``).
    
    Example
    -------
    >>> asm = AdvancedStepperMotorControl()         # init object
    >>> asm.walk_bursts('fwd', 100)                 # walk in a direction
    >>> asm.turn_around('fwd_to_back')              # do a uturn.
    
    
    """
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self.turn_dct = TURN_AROUND_STEP_SEQUENCE.copy()
        
    def _check_direction_change(self, dir_change):
        """check if given direction change is valid. Raises 
        AdvancedStepperMotorException otherwise
        """
        msg = "Passed direction chane {0} is not valid. Try instead: {1}".format(
                dir_change,
                self.turn_dct.keys()
        )
        
        if not dir_change in self.turn_dct.keys():
            raise AdvancedStepperMotorException(msg)        
    
    def turn_around(self, dir_change):
        """performs a uturn along specified direction change.
        
        The uturns performed by this functions are used to avoid a hystersis 
        effect in motion due to sudden direction changes with the piezo coarse
        motion motor. This function only is able to correct for hysteresis 
        effects from direction changes of 180 degree. It uses the calibration 
        values from the ``TURN_AROUND_STEP_SEQUENCE`` dictionary.

        Parameter
        --------
        dir_change | str
            specified the direction change (e.g. 'fwd_to_back'). 
            
        Raises
        ------
        AdvancedStepperMotorException 
            if given dir_change is not a key word argument of 
            ``TURN_AROUND_STEP_SEUQ``.
        
        """
        self._check_direction_change(dir_change)
        
        for step_sequ in self.turn_dct.get(dir_change):
            self.walk_bursts(*step_sequ)
        return
    
    def turn_around_from(self, direction):
        """performs a uturn starting from specifed direction.
        
        This function uses the TURN_AROUND_FROM dictionary to map the specified
        direction to a *direction change* (e.g. maps *left* to *left_to_right*).
        Check out the ``turn_around()`` function for a detailed procedure 
        description
        
        Parameter:
        ----------
        direction | str
            needs to be a valid stepper motor direction
            
        Raises
        ------
        StepperMotorException
            if given direction is not valid.
            
        """
        self._check_motor_direction(direction)
        
        dir_change = TURN_AROUND_FROM[direction]
        self.turn_around(dir_change)
        return
        

    def conv_one_step(self, conv_to, step_nr):
        """converts give step_nr to corresponding number along other direction 
        
        This function converses step number along a given direction to step 
        numbers resulting in the same distance travelled along the opposite 
        direction. Therefore this function helps to overcome the problem that 
        the piezo coarse motion step width is not the same for different 
        directions. E.g.: 100 steps in ´´back´´ direction correspond not to the 
        same distance as walking 100 steps in ´´fwd´´ direction. The 
        calibration values are stored in a dictionary.
        
        Parameter:
        ----------
        conv_to | str
            specifies direction into which steps will be conveted. Needs to be 
            a valid direction name.
        step_nr | int
            number of steps to be converted. For the function to be applied 
            correctly, this value needs to refer to the number of steps towards 
            the opposite direction as specified in the ´´conv_to´´ parameter.
    
        Returns:
        --------
        int |
            step number 
    
    
        Example:
        --------
        >>> step_nr_back = 100
        >>> step_nr_fwd = conv_one_steps('fwd', step_nr_bacjk)
        >>> step_nr_fwd
        ... 198                 # this value depends on the calibration factor
    
    
        Raises:
        -------
        ValueError
            if ´´conv_to´´ is not a valid direction name.
            
            
        """
        err_msg = 'conv_to: {} is not a valid argument. Try instead: {}'.format(
            conv_to, list(CONV_TO.keys())
        )
        if not conv_to in CONV_TO.keys():
            raise ValueError(err_msg)

        ratio = CONV_TO[conv_to]
        return int(step_nr*ratio)
    

    def conv_steps(self, conv_to, step_vec):
        """converts a vector of step values to corresponding number along 
        specified direction
        
        
        Parameter:
        ----------
        conv_to | str
            specifies direction into which steps will be conveted. Needs to be 
            a valid direction name.
        step_nr | int
            number of steps to be converted. For the function to be applied 
            correctly, this value needs to refer to the number of steps towards 
            the opposite direction as specified in the ´´conv_to´´ parameter.
    
        
        Example:
        --------
        >>> step_vec_back = [100,200,300]
        >>> step_vec_fwd = conv_steps('fwd', step_nr_back)
        >>> step_vec_fwd
        ... [229, 458, 687]   # this value depends on the calibration factor
        
    
        Raises:
        -------
        ValueError
            if ´´conv_to´´ is not a valid direction name.
    
        """
        step_vec_ = np.array(
            [self.conv_one_step(conv_to, step_nr) for step_nr in step_vec],
            dtype = np.int
        )
        return step_vec_

        
# ===========================================================================
# init hardware
# ===========================================================================

## init u3
#d = u3.U3()
#d.configU3()
#
#
#
## init DI channels
#d.getDIState(5)
#d.getDIState(6)
#d.getDIState(7)
#
## init DO channels
#d.setDOState(0, state=1)
#d.setDOState(1, state=1)
#d.setDOState(2, state=1)

