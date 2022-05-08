
# Rosmo ESC firmware

An open source firmware based on [SimpleFOC](www.simplefoc.com) for [Rosmo](https://rosmo.io) robot's ESC board. The ESC board, like all of Rosmo, is open source and the design can be found [here](https://github.com/rosmo-robot/Rosmo_ESC).

:exclamation: Work in progress! Running, basic motor control working, but very much still under development. See checkmarks in feature list below.

## Features

- :white_check_mark: for STM32G4, specifically the STM32G491MET6, the MCU used on Rosmo ESC v1
- :white_check_mark: 2x motor driver support in 6-PWM configuration
- :white_check_mark: FOC - field oriented control of 2 motors
- voltage, position, velocity or torque control modes
- current sensing for 2 motors
- USB support :negative_squared_cross_mark: for programming, :white_check_mark: serial output and :negative_squared_cross_mark: tuning control
- :white_check_mark: m5 bus header connector to connect to m5 stack ecosystem
- :white_check_mark: control via I2C or USB Serial port
- compatible with SimpleFOC studio
- support for 3 SPI connections
    - 2 with SH-1.0 ports for the motor encoder sensors
    - one attached to m5 bus header SPI
    - for m5 bus: 2 nCS pins selectable in hardware via jumpers
- support for 2 I2C connections
    - :white_check_mark: one on m5 bus header I2C
    - one for grove I2C port
    - :white_check_mark: 4 I2C addresses selectable in hardware via jumpers
- :white_check_mark: ARM-10 debug header with SWD debug/programming support via STLink programmer (required to flash newly produced boards)
- deep sleep support and wake from m5 bus support (2 pins selectable in hardware via jumpers)
- :white_check_mark: written in C++ for Arduino framework using the PlatformIO embedded development environment

## Setup

### First time installation

If you got your Rosmo ESC from me, it is already flashed with a version of this firmware to get you started quickly. You can proceed directly to the next step, *Hardware setup*.

If you made your own Rosmo ESC boards, they will need to be flashed with this firmware before they can be used. Please skip down to the *First firmware download* section below.

### Hardware setup

Please follow the Rosmo assembly instructions, and pay attention to the following:

- Don't connect anything to power (battery) yet. Please wait until asked to do so in the *Calibration & tuning* step, or risk damage to ESC and/or motor.
- motors, magnets and sensors must be carefully installed and well aligned. Misalignment of the magnets or sensors will affect the motor's performance.
- motor leads must be connected to Rosmo ESC. 3-pin pluggable terminal blocks are used for this, so all you need is a small flat-head screwdriver.
- sensor cables must be connected to Rosmo ESC using the SPI ports and JST-SH-1.0 6-pin cables.
- Power (the battery) will only later be connected to the ESC, a 2-pin pluggable terminal block is used. You can prepare the power cabling, but don't plug in the terminal block yet!
- :warning: Carefully check power cable polarity - the ESC does NOT have reverse polarity protection, and connecting the battery incorrectly will likely destroy it!
- Please take care to keep cables tidy, and in particular to make sure that no short-circuits are possible (e.g. through uninsulated cables touching ESC board, battery terminals or similar). Insulate exposed cables with electrical tape or heat shrink, cover exposed battery terminals, etc...
- At the point in the instructions where it asks you to calibrate the motors (before attaching treads), please continue with the next step, *Calibration & tuning*.

Notes:

If you're not building a Rosmo, that's fine, the ESC will work with most smallish brushless motors (up to 5A).<br/>
You can run the ESC without sensors in open-loop control mode, but this isn't nearly as good as having sensors and field-oriented control.<br/>
If using larger motors than those designed for Rosmo robot, the ESC may need a heat sink and/or cooling on its driver ICs for sustained operation.

### Calibration & tuning

When your Rosmo is mostly assembled, but *before* the tracks are attached, find a place where you can put him close to a PC.

- Make sure you're ready. You will need:
    - a charged battery (but don't connect it to the ESC yet!)
    - a USB-C cable from your PC to Rosmo ESC
- Put Rosmo up onto something so that the motor wheels can turn freely, but Rosmo is stable and won't wobble or fall.
- Connect your PC/Laptop to the Rosmo ESC using the USB-C cable. You should immediately see a small green LED light up to the left of the USB port.
- TODO


## Firmware updates

TODO

## Initial firmware flashing

You will need:

- STLink
- Cable to connect to ARM-10 or STLink-14 header in 1.27mm pitch

:information_source: a STLink V3 Mini comes with the right cable, and will work out of the box

To flash the RosmoESC for the first time:

 1. Clone/Download this project, open it in PlatformIO and make sure it compiles. You will need the STM32 platform and the SimpleFOC Library and SimpleFOC Drivers Library as dependencies.
 1. If its the first time you're using your ST-Link, follow its instructions to install any necessary drivers for your OS.
 1. Connect the STLink to the RosmoESC. Pay careful attention to the orientation of the plug. (A normal ST-Link V3 Mini cable will have the cable facing towards the M5-Stack header when attached.)
 1. Connect the STLink to USB. It's power indication LED should come on.
 1. Connect the RosmoESC to USB to give it power
 1. Using PlatformIO, download this project to the RosmoESC.
 1. Open PlatformIO's Serial Monitor, using the RosmoESC's USB serial connection. You should see debug output from the RosmoESC.

