# Navigate and wait example

import math
import rospy
from coptra import srv
from coptra.srv import SetMode, Arm, Takeoff
from std_srvs.srv import Trigger

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

def navigate_wait(x=0, y=0, z=0, yaw=math.nan, speed=0.5, frame_id='body', tolerance=0.2):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)

print('Setting mode to GUIDED...')
set_mode(mode='GUIDED')

print('Arming drone...')
arm(arm=True)

print('Taking off to 1 meter...')
takeoff(altitude=1.0)

print('Take off 1 meter')
navigate_wait(z=1, frame_id='body')

print('Fly forward 1 m')
navigate_wait(x=1, frame_id='body')

print('Land')
land()
