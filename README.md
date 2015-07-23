# Snotbot

This repository contains the files necessary for sending a drone out to sea to collect whale snot.

## Installation
1. Ensure you have ROS installed (preferablly Jade). Follow the directions [here] (http://wiki.ros.org/jade/Installation/Ubuntu).
2. Install a package to allow [joystick control](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick).
3. Install [MavROS](https://github.com/mavlink/mavros/tree/master/mavros).
4. Clone this repository in the /src folder of your Catkin workspace

## Hardware
The main components of our system includes the [3DR Pixhawk](https://store.3drobotics.com/products/3dr-pixhawk/?utm_source=google&utm_medium=cpc&utm_term=branded&utm_campaign=branded&gclid=CjwKEAjwocKtBRCf9d_Q5ovcyHASJAAHhJYOfuoWZGXrE88SaWO78lJ5QHvzFXxYLjEovl3Xv93cwRoCKv3w_wcB), the [Extreme 3D Pro Joystick](http://gaming.logitech.com/en-us/product/extreme-3d-pro-joystick), the [Connex video link](http://connex.amimon.com/), the [INOGENI 4K2USB3 video capture card](http://www.bhphotovideo.com/c/product/1073122-REG/inogeni_4k2usb3_4k_hdmi_to_usb.html) and a GoPro HERO 4 (http://shop.gopro.com/hero4/hero4-black/CHDHX-401.html). In order to communicate with the Pixhawk, a [915 MHz radio set](https://store.3drobotics.com/products/3dr-radio-set) is required.

#### Setup
Three USB ports are needed to fully utilize the code:
1. Joystick
2. 915 MHz radio
3. Connex ground station (video link)
  - Power port - ~12V
  - 5 antennae
  - HDMI port --> INOGENI 4k2USB3 --> ground station computer

After plugging in the radio for telemetry, type in the following to write permissions to the USB port, so that MavROS can properly communicate with the air vehicle:
`sudo chmod a+rw /dev/ttyUSB0`

Then, power up the drone and press the arming button.

## Usage
Type in the following to run the code: `roslaunch snotbot snotbot.launch`

The joystick can be used to send specific commands to the air vehicle:
#### Buttons
- 1 (trigger) - Failsafe mode: allows user to fully control the drone using the joystick's axes
- 3 - Arm
- 4 - Disarm
- 5 - Initiate autonomy routine: starts the Snotbot mission
- 6 - End autonomy routine: return to launch, land, and disarm
