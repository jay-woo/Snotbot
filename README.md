# Snotbot

This repository contains the files necessary for sending a drone out to sea to collect whale snot.

## Installation
1. Ensure you have ROS installed (preferablly Jade). Follow the directions [here] (http://wiki.ros.org/jade/Installation/Ubuntu).
2. Install a package to allow [joystick control](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick).
3. Install [MavROS](https://github.com/mavlink/mavros/tree/master/mavros).
4. Clone this repository in the /src folder of your Catkin workspace

## Hardware
The main components of our system includes the [3DR Pixhawk](https://store.3drobotics.com/products/3dr-pixhawk/?utm_source=google&utm_medium=cpc&utm_term=branded&utm_campaign=branded&gclid=CjwKEAjwocKtBRCf9d_Q5ovcyHASJAAHhJYOfuoWZGXrE88SaWO78lJ5QHvzFXxYLjEovl3Xv93cwRoCKv3w_wcB), the [Extreme 3D Pro Joystick](http://gaming.logitech.com/en-us/product/extreme-3d-pro-joystick), the [Connex video link](http://connex.amimon.com/), and a GoPro HERO 4 (http://shop.gopro.com/hero4/hero4-black/CHDHX-401.html). In order to communicate with the Pixhawk, a [915 MHz radio set](https://store.3drobotics.com/products/3dr-radio-set) is required.
