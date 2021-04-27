# ltstm-annealing-controller
Scripts to automatize the annealing process on the LTSTM


## Requirements

### Power supply control
Scripts use the Labjack U3 for communication with the DC power supply. To make the U3 available within Python the Labjack *exodriver* and *LabjackPython* need to be installed. Check the Labjack page for more information:
* [exodriver ](https://labjack.com/support/software/installers/exodriver)
* [LabjackPython](https://pypi.org/project/LabJackPython/)


### Pressure gauge control
communication with the Pfeiffer PKR251 pressure gauge requires ``pyserial`` to be installed. Base class for Pressure gauge control is based on code from [PyExpLabSys](https://pyexplabsys.readthedocs.io/drivers/pfeiffer.html).




