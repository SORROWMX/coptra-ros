# Get telemetry example

import rospy
from coptra import srv

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

# Print drone's state
print(get_telemetry())
