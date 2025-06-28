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
* Ability to configure the profile velocity (`0x6081`).
* Ability to read the torque actual value (`0x6077`).
* Ability to read the velocity actual value (`0x606C`).
* Servo ON/OFF services to control motor power state.


### Reusable CI Workflow

This repository exposes a reusable GitHub Actions workflow at
`.github/workflows/ci.yml`. Other ROS 2 packages can invoke it using
`workflow_call`:

```yaml
name: CI
on:
  pull_request:
jobs:
  build-and-test:
    uses: <owner>/motion-control-mecanum/.github/workflows/ci.yml@main
    with:
      repo-path: <your-package-name>
```
