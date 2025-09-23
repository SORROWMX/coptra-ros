# Flight example

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

print('Setting mode to GUIDED...')
set_mode(mode='GUIDED')

print('Arming drone...')
arm(arm=True)

print('Taking off to 1 meter...')
takeoff(altitude=1.0)

print('Take off and hover 1 m above the ground')
navigate(x=0, y=0, z=1, frame_id='body')

# Wait for 5 seconds
rospy.sleep(5)

print('Fly forward 1 m')
navigate(x=1, y=0, z=0, frame_id='body')

# Wait for 5 seconds
rospy.sleep(5)

print('Perform landing')
land()
