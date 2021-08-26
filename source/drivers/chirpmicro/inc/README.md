# Chirp SonicLib

## Summary

Chirp SonicLib is a set of API functions and sensor driver routines designed 
to easily control Chirp ultrasonic sensors from an embedded C application.  It allows an
application developer to obtain ultrasonic range data from one or more devices, without
needing to develop special low-level code to interact with the sensors directly.

The SonicLib API functions provide a consistent interface for an application to use
Chirp sensors in various situations.  This is especially important, because
all Chirp sensors are completely software-defined, including the register map.  The
SonicLib interfaces allow an application to use different Chirp sensor firmware images,
without requiring code changes.  Only a single initialization parameter must be modified,
and one line added to a header file, to use a new sensor firmware version.

\note All operation of the sensor is controlled through the set of functions, data structures,
and symbolic values defined in the soniclib.h header file.  You should not need to modify this file 
or the SonicLib functions, or use lower-level internal functions such as described in 
the ch_driver.h file.  Using any of these non-public methods will reduce your ability to 
benefit from future enhancements and releases from Chirp.

## SonicLib API
The main documentation for the SonicLib API may be found in the soniclib.h header file.
That file contains definitions of all public interfaces that applications 
use to interact with the sensor.

## Required Board Support Package
SonicLib also defines a set of board support package (BSP) functions that
are specific to the hardware platform being used.  The chirp_bsp.h file
contains the definitions of these hardware interfaces.  These interfaces
allow the standard SonicLib functions to interact with the peripheral
devices and other resources on the board.  The BSP implementation is NOT
part of SonicLib, and must be provided by the developer, board vendor, 
or Chirp.  Contact Chirp for more information on available BSPs.

## Chirp Driver (internal)
SonicLib includes internal driver functions to interact directly with the sensor.
These modules are distributed as source code, to simplify integration with your embedded
application and for reference only.  These functions are described in the ch_driver.h 
header file.  You should not modify these functions or call them directly from your application.

## Support
support@chirpmicro.com

## Copyright
&copy; Copyright 2016-2020, Chirp Microsystems.  All rights reserved.
