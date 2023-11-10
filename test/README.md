
This directory is intended for PlatformIO Test Runner and project tests.

Unit Testing is a software testing method by which individual units of
source code, sets of one or more MCU program modules together with associated
control data, usage procedures, and operating procedures, are tested to
determine whether they are fit for use. Unit testing finds problems early
in the development cycle.

More information about PlatformIO Unit Testing:
- https://docs.platformio.org/en/latest/advanced/unit-testing/index.html


## MCU pins

cs	z+/h3	PB8
mosi	b+/h2	PB7
clk	a+/h1	PB6

miso	J3 PWM	PA15


##

Notes:
Does something override the pin definition? 
Or why does else does the sensor only allow single read? 
--> something in the sensor.init method...