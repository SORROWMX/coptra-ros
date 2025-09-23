
# ArUco marker navigation example
#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong
from mavros_msgs.msg import State
from pymavlink import mavutil
from coptra import srv
from coptra.srv import Takeoff, Arm, SetMode
from std_srvs.srv import Trigger
rospy.init_node('flight')
TAKEOFF_ALT = 1.0
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
takeoff = rospy.ServiceProxy('takeoff', Takeoff)
arm = rospy.ServiceProxy('arm', Arm)
set_mode = rospy.ServiceProxy('set_mode', SetMode)
land = rospy.ServiceProxy('land', Trigger)
def takeoff_drone(altitude: float):
    res = takeoff(altitude=altitude)
    return res.success
def arm_drone(arm_value: bool):
    res = arm(arm=arm_value)
    return res.success
def set_mode_drone(mode: str):
    res = set_mode(mode=mode)
    return res.success

def navigate_wait(x=0, y=0, z=0, yaw=math.nan, speed=0.1, frame_id='body', tolerance=0.2):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)
def main():
    # Режим GUIDED, ARM, взлёт на заданную высоту
    if not set_mode_drone('GUIDED'):
        rospy.logerr('Failed to switch to GUIDED')
        return
    if not arm_drone(True):
        rospy.logerr('Arm failed')
        return
    if not takeoff_drone(TAKEOFF_ALT):
        rospy.logwarn('Takeoff command not acknowledged, waiting by altitude...')

    # Wait for 5 seconds
    rospy.sleep(3)

    print('Fly 1 meter above ArUco marker 10')
    navigate_wait(x=1, y=2, z=1.1, frame_id='aruco_map')
    navigate_wait(x=2, y=2, z=1.1, frame_id='aruco_map')
    # Wait for 5 seconds
    rospy.sleep(5)


    print('Perform landing')
    land()
if __name__ == '__main__':
    main()
