# py-dynamixel
Python wrapper for Dynamixel SDK. Uses dynamixel SDK, but with a more straightforward API. Attempts to do something like pypot.


# Installation of dependencies
This library is dependant on the dynamixel SDK. S please install the dynamixel SDK first. For python, this can be done using
```pip3 install dynamixel-sdk```


Currently only supports XM430-W350T. but you can change the control table in the src/io.py file for your corresponding motor. Also need to change the conversions for your correspoding motor from degree to pulses and backwards (just check the number of pulses per 360 degrees for your motor model).