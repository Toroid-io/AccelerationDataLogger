# AccelerationDataLogger

**AccelerationDataLogger** provides a hardware-software bundle to
measure and log accelerations. Originally developed to measure
oscillations in engineering models, it may be used in the following
applications, among others:

 - Structural dynamics
 - Sports science
 - Simple vibration analysis
 - Fall detection
 - Wearables development
 - As an accelerometer playground

The board is capable of measuring **two accelerations simultaneously**
with a maximum sampling frequency of 800Hz, during 11 seconds. Longer
measurements are possible reducing the sampling rate, or using only one
accelerometer.

It currently uses jellybean parts as accelerometers. The board currently
supports the following breakout boards:
 - MPU6050 (GY-521 breakout)
 - ADXL345 (GY-291 and Sparkfun breakouts)

## Quick user manual
- Power the board using batteries or the USB cable.
- Fix the board into the structure or person.
- Calibrate the board by pressing the *Calibrate button*.
- Acquire the acceleration by pressing the *Acquire button*.
- Connect the board to your computer to download the data.

## The Hardware
You may find the KiCad schematic and board in the `Board` directory.

### Overview

The board does all the acquisition and storing procedure autonomously.
Hence, it may be battery powered during acquisition, and then connected
to a computer to download the data. In the current state, the board
requires at least 6.8V to work autonomously (this tension may be
provided by two LiPo cells or a 9V battery).

It has three pushable buttons, in order to reset the system, calibrate
the device and acquire data. A pushable switch enables or disables the
battery for autonomous mode.

The board may be connected to a computer using a USB micro-B cable.

That's all you need to know about the hardware!

### Memory

The board has 1MBit of RAM, where all the data is stored. Given that a
16-bit accelerometer provides 6 bytes of data for each measurement (plus
1 byte for the accelerometer ID), the board is capable of saving 18724
measurements.

That equals:
 - 11.7 seconds using two accelerometers at 800Hz
 - 23.4 seconds using one accelerometer at 800Hz
 - 6.24 minutes with one accelerometer at 50Hz
 - ...

## The Firmware

ChibiOS RTOS is used for the firmware. Tasks are used to separate the
acquisition of each sensor, write the RAM, and send measurements when
requested. Give a look into the `Firmware` folder.

## The acquisition software

A simple interface has been written in Qt C++, in order to download and
preview the data. The interface looks like this:

![Software screenshot](https://github.com/Toroid-io/AccelerationDataLogger/raw/dev/Qt/example.png)

Currently, the interface only allows to connect and download the data
from the board. Future versions will allow the modification of some
parameters, such as the sampling frequency.

## I want one!

### Build One

![Bare PCB](https://644db4de3505c40a0444-327723bce298e3ff5813fb42baeefbaa.ssl.cf1.rackcdn.com/3cf1cb30e9e4993c49112f53ba406c36.png)

You can build your own AccelerationDataLogger. Mounting the PCB requires
average soldering skills (0603 footprints and LQFP packages), but it is
the cheap way to go.

You can get the PCB from [OSH
Park](https://oshpark.com/shared_projects/0VAEaJ3K), or generate your
own Gerbers from the PCB. The BOM is in the `Board` directory.

### Get one built

We can send you a populated PCB. Just contact us at `admin [at]
toroid.io` for a quote.

## Development

### Contribute

Do you want to add or request a new feature? Did you find a bug in the
code? Create an issue or a pull request! All contributions are welcome.

### Future features

A non-exhaustive list of future features:
 - Add support to more accelerometers
 - Use a coin cell to power the system autonomously
 - Reduce the size of the board
 - Implement other MEMS features (DMP, fall detection)
