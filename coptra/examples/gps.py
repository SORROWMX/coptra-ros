# GPS navigation example

import rospy
from coptra import srv
from coptra.srv import SetMode, Arm, Takeoff
from std_srvs.srv import Trigger
import math

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
set_mode = rospy.ServiceProxy('set_mode', SetMode)
arm = rospy.ServiceProxy('arm', Arm)
takeoff = rospy.ServiceProxy('takeoff', Takeoff)
land = rospy.ServiceProxy('land', Trigger)

# GPS arrival waiting function
def wait_arrival(tolerance=0.2):
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

start = get_telemetry()

if math.isnan(start.lat):
    raise Exception('No global position, install and configure GPS sensor')

print('Start point global position: lat={}, lon={}'.format(start.lat, start.lon))

print('Setting mode to GUIDED...')
set_mode(mode='GUIDED')

print('Arming drone...')
arm(arm=True)

print('Taking off to 3 meters...')
takeoff(altitude=3.0)

print('Take off 3 meters')
navigate(x=0, y=0, z=3, frame_id='body')
wait_arrival()

print('Fly 1 arcsecond to the North (approx. 30 meters)')
navigate_global(lat=start.lat+1.0/60/60, lon=start.lon, z=start.z+3, yaw=math.inf, speed=5)
wait_arrival()

print('Fly to home position')
navigate_global(lat=start.lat, lon=start.lon, z=start.z+3, yaw=math.inf, speed=5)
wait_arrival()

print('Land')
land()
