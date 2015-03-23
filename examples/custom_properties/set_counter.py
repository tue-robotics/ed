#! /usr/bin/python

import rospy
from ed.srv import UpdateSrv

rospy.wait_for_service('ed/update')

update = rospy.ServiceProxy('ed/update', UpdateSrv)
print update('{"entities": [{"id":"test-entity", "properties":[{"name":"counter", "value": 1000}]}]}')

