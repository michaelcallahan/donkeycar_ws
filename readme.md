# DonkeyCar ROS2 Workspace

This repository contains node modules and library code to create a DonkeyCar robotics application using the ROS2 framework, as an alternative to the existing DonkeyCar Python project.

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
<!---
- [Workspace Setup](#workspace-setup)
- [Building the Workspace](#building-the-workspace)
- [Running the Application](#running-the-application)
- [Nodes Description](#nodes-description)
- [Arduino Setup](#arduino-setup)
- [License](#license)
-->

## Overview

This workspace (`donkeycar_ws`) contains ROS2 packages that replace the traditional DonkeyCar Python-based implementation. The main goal is to leverage ROS2 capabilities for better modularity, communication, and scalability in building and operating the DonkeyCar.

## Installation

### Prerequisites

- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html) or later
- [colcon](https://colcon.readthedocs.io/en/released/user/installation.html) build tool
- `optional` Arduino IDE for programming Arduino I2C checking tool

### Additional Libraries

- [micro-ROS](https://micro.ros.org/) for Arduino
- `i2c-tools` for checking I2C devices

### Enable I2C on Raspberry Pi

Make sure I2C is enabled on your Raspberry Pi

### Connecting a controller/gamepad

Using the ros joy library, it is possible to connect to a bluetooth controller and map the car controls to the joystick of the controller.

Connect to controlelr using bluetoothctl:
```bluetoothctl agent on```
```bluetoothctl scan on```
```bluetoothctl pair []```
```bluetoothctl connect []```

