# Motion Control Mecanum Package

This package provides a simple example of a mecanum wheel motion controller for ROS 2.

### New Features

* Support for setting the DS402 *Modes of Operation* object (`0x6060`).
* Ability to command target velocity using Profile Velocity Mode (`0x60FF`).
* Ability to configure the velocity threshold (`0x606F`).
* Ability to configure the velocity window (`0x606D`).
* Ability to configure the quick stop option code (`0x605A`).
* Ability to configure the quick stop deceleration (`0x6085`).
* Ability to configure the maximum torque limit (`0x6072`).
* Ability to configure the profile acceleration (`0x6083`).
* Ability to configure the profile deceleration (`0x6084`).
* Ability to configure the end velocity (`0x6082`).
