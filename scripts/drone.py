class Drone():
    def __init__(self, drone_id):
        # Non-ROS variables
        self.drone_id = drone_id
        (self.x, self.y, self.z, self.yaw, self.cam_tilt) = (1500, 1500, 1000, 1500, 1300)
        self.mode = 0
      
        # ROS state variables
        self.armed = False
        self.flight_mode = ''
        self.voltage = 0
        self.current = 0
        self.battery_remaining = 0
        self.latitude = None
        self.longitude = None
        self.altitude = 0.0
        self.waypoints = []

    def state_callback(self, data):
        self.armed = data.armed
        self.flight_mode = data.mode

    def battery_callback(self, data):
        self.voltage = data.voltage
        self.current = data.current
        self.battery_remaining = data.remaining

    def gps_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude

    def altitude_callback(self, data):
        self.altitude = data.data

    def waypoints_callback(self, data):
        self.waypoints = data.waypoints
