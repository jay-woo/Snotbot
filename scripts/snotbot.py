#!/usr/bin/env python
import rospy
import roslib
import time
import math
roslib.load_manifest('mavros')

from geometry_msgs.msg import Point
from std_msgs.msg import Header, Float64, UInt8
from sensor_msgs.msg import Joy, NavSatFix
from mavros.msg import BatteryStatus, State, OverrideRCIn, Waypoint, WaypointList
from mavros.srv import CommandBool, WaypointPush, WaypointClear, WaypointGOTO, SetMode

from drone import *
import mission_parser

joystick = {'arm': 2, 'disarm': 3, 'failsafe': 0, 'auto': 4, 'manual': 5, 'x': 0, 'y': 1, 'z': 3, 'yaw': 2}
xbox     = {'arm': 0, 'disarm': 1, 'failsafe': 8, 'auto': 2, 'manual': 3, 'x': 3, 'y': 4, 'z': 1, 'yaw': 0}
ctrl = xbox

class Snotbot():
    ###
    # Initializes drone control variables
    ### 
    def __init__(self):
        # Initializes ROS node
        rospy.init_node('snotbot')

        # Joystick variables
        self.axes = []
        self.buttons = []
        self.controller = xbox

        # Drone variables
        self.drone = Drone(0)  # Drone class contains valuable functions/variables

        # Landing platform variables
        self.platform_gps = (0, 0, 0)

        # Vision variables
        self.fiducial = (0, 0, 0) 

        # Miscellaneous variables
        self.time_mode_started = 0    # Records when each mode in the finite state machine started
        self.target_alt = 3.0
        self.failsafe = True 
        self.launched = False
        self.returning = False
        self.just_armed = True

        # ROS publishers
        self.pub_rc = rospy.Publisher('/drone/rc/override', OverrideRCIn, queue_size=10)
        self.pub_mode = rospy.Publisher('mode', UInt8, queue_size=10)

        # ROS subscribers
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.sub_vision = rospy.Subscriber('/vision', Point, self.vision_callback)
        self.sub_platform_gps = rospy.Subscriber('/platform_gps', Point, self.platform_gps_callback)

        self.sub_state = rospy.Subscriber('/drone/state', State, self.drone.state_callback)
        self.sub_battery = rospy.Subscriber('/drone/battery', BatteryStatus, self.drone.battery_callback)
        self.sub_drone_gps = rospy.Subscriber('/drone/gps/fix', NavSatFix, self.drone.gps_callback)
        self.sub_altitude = rospy.Subscriber('/drone/global_position/rel_alt', Float64, self.drone.altitude_callback)
        self.sub_waypoints = rospy.Subscriber('/drone/mission/waypoints', WaypointList, self.drone.waypoints_callback)

        # ROS services
        self.srv_arm = rospy.ServiceProxy('/drone/cmd/arming', CommandBool)
        self.srv_mode = rospy.ServiceProxy('/drone/set_mode', SetMode)
        self.srv_wp_push =  rospy.ServiceProxy('/drone/mission/push', WaypointPush)
        self.srv_wp_clear = rospy.ServiceProxy('/drone/mission/clear', WaypointClear)
        self.srv_wp_goto = rospy.ServiceProxy('/drone/mission/goto', WaypointGOTO)

        # Preparing parameters
        print "Waiting for drone to connect..."
        for i in reversed(range(10)):
            print str(i+1) + "..."
            time.sleep(1)
        print "Ready"

        # Main loop
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.fly()
            r.sleep()

    ###
    # Retrieves joystick data
    ###
    def joy_callback(self, data):
        self.axes = list(data.axes)
        self.buttons = list(data.buttons)

    ###
    # Retrieves vision data
    ###
    def vision_callback(self, data):
       self.fiducial = (data.x, data.y, data.z)

    ###
    # Retrieves GPS data from platform
    ###
    def platform_gps_callback(self, data):
        self.platform_gps = (data.x, data.y, data.z)

    ########################
    # FINITE STATE MACHINE #
    ########################

    ###
    # Mode 1: arms the drone
    ###
    def arm(self):
        self.drone.z = 1000

        # Arming process
        if not self.drone.armed:
            self.srv_mode(0, '5') # Loiter
            self.srv_arm(True)

        # Waits a few seconds before switching to launch mode
        if millis() - self.time_mode_started > 3000:
            self.time_mode_started = millis()
            self.srv_mode(0, '3')
            self.drone.mode = 2

    ###
    # Mode 2: launches the drone to a certain altitude
    ###
    def launch(self):
        self.drone.z = 1250

        # Launches the drone at the current location
        if not self.launched:
            waypoints = mission_parser.takeoff_waypoints(self.target_alt)           
            self.srv_wp_push(waypoints)
            self.launched = True

        if abs(float(self.drone.altitude) - self.target_alt) < 0.5:
            self.time_mode_started = millis()
            self.srv_mode(0, '5')
            self.drone.mode = 3

    ###
    # Mode 3: waits for user to select object to track
    ###
    def track(self):
        self.drone.x = 1500 - self.axes[0] * 300
        self.drone.y = 1500 - self.axes[1] * 300
        self.drone.z = 1500

        if millis() - self.time_mode_started > 10000:
            (self.drone.x, self.drone.y, self.drone.z) = (1500, 1500, 1500)
            self.time_mode_started = millis()
            self.srv_mode(0, '3')
            self.drone.mode = 4

    ###
    # Mode 4: return to launch
    ###
    def rtl(self):
        if not self.returning:
            (lat, lon) = (self.platform_gps[0], self.platform_gps[1])
            hold_time = 10
            platform_wp = mission_parser.make_global_waypoint(lat, lon, self.target_alt, hold_time)
            platform_wp = [platform_wp, platform_wp]
            self.srv_wp_push(platform_wp)
            self.returning = True

        current_waypoints = self.drone.waypoints
        at_platform = False
        for i in range(len(current_waypoints)):
            if current_waypoints[i].is_current and i == 1:
                at_platform = True

        if at_platform:
            self.time_mode_started = millis()
            self.srv_mode(0, '5')
            self.drone.mode = 5

    ###
    # Mode 5: land
    ###
    def land(self):
        self.z = 1450

        if millis() - self.time_mode_started > 15000:
            self.time_mode_started = millis()
            self.drone.mode = 6
        elif millis() - self.time_mode_started > 10000:
            self.z = 1000

    ###
    # Mode 6: disarm
    ###
    def disarm(self):
        self.srv_arm(False)
        self.drone.mode = 0

    ###
    # Publishes RC commands to the vehicle, depending on what mode it is currently in
    ###
    def fly(self): 
        if self.buttons:
            # Button 1 - enters failsafe mode (enables full control)
            if self.buttons[ ctrl['failsafe'] ]:
                if self.failsafe:
                    print "Failsafe: OFF"
                else:
                    print "Failsafe: ON"
                self.failsafe = not self.failsafe 

            # Button 3 - arms the drone
            if self.buttons[ ctrl['arm'] ]:
                self.srv_arm(True)
                print "Arming drone"

            # Button 4 - disarms drone
            if self.buttons[ ctrl['disarm'] ]:
                self.srv_arm(False) 
                print "Disarming drone"

            # Button 5 - initiate autonomy routine
            if self.buttons[ ctrl['auto'] ]:
                self.drone.mode = 1
                self.time_mode_started = millis()
                print "Beginning finite state machine"
                
            # Button 6 - end autonomy routine (RTL)
            if self.buttons[ ctrl['manual'] ]:
                self.drone.mode = 5
                print "Returning to launch and landing"

        # Initiates finite state machines
        if not self.failsafe:
            # Arming
            if self.drone.mode == 1:
                self.srv_mode(0, '5')
                self.arm()

            # Launching
            if self.drone.mode == 2:
                self.launch()

            # Tracking
            if self.drone.mode == 3:
                self.track()

            # Returning to launch
            if self.drone.mode == 4:
                self.rtl()

            # Landing
            if self.drone.mode == 5:
                self.land()

            # Disarming
            if self.drone.mode == 6:
                self.disarm()

        else:
            # Reads joystick values
            self.srv_mode(0, '5')
            self.drone.mode = 0

            x = 1500 - self.axes[ ctrl['x'] ] * 300
            y = 1500 - self.axes[ ctrl['y'] ] * 300
            z = 1500 + self.axes[ ctrl['z'] ] * 500
            yaw = 1500 - self.axes[ ctrl['yaw'] ] * 300

            if self.just_armed:
                z = 1000
                if abs(self.axes[ ctrl['z'] ]) > 0.1:
                    self.just_armed = False

            if z < 1100:
                z = 1000
            elif z < 1450:
                z = 1350
            elif z > 1600:
                z = 1600
            else:
                z = 1500

            if abs(x - 1500) < 50:
                x = 1500
            if abs(y - 1500) < 50:
                y = 1500

            print (x, y, z)

            (self.drone.x, self.drone.y, self.drone.z, self.drone.yaw) = (x, y, z, yaw)

        # Publishes commands
        if self.drone.armed:
            print self.drone.mode
            rc_msg = OverrideRCIn()
            rc_msg.channels = [self.drone.x, self.drone.y, self.drone.z, self.drone.yaw, 1400, 0, 0, self.drone.cam_tilt] 
            self.pub_rc.publish(rc_msg) 

        self.pub_mode.publish(self.drone.mode)

###
# Calculates the number of milliseconds since the epoch
###
def millis():
    return int(round(time.time() * 1000))

if __name__ == '__main__':
    try:
        var = Snotbot()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
