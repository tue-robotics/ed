#! /usr/bin/python

import rospy
from ed.srv import UpdateSrv

rospy.wait_for_service('ed/update')

update = rospy.ServiceProxy('ed/update', UpdateSrv)

req = """{
        "entities": [
            {
                "id": "test-entity",
                "idx": 0,
                "properties": [
                    {
                        "name": "pose",
                        "pos": {
                            "x": 0,
                            "y": 0,
                            "z": 0
                        },
                        "rot": {
                            "xx": 1,
                            "xy": 0,
                            "xz": 0,
                            "yx": 0,
                            "yy": 1,
                            "yz": 0,
                            "zx": 0,
                            "zy": 0,
                            "zz": 1
                        }
                    },
                    {
                        "name": "counter",
                        "value": 0
                    }
                ]
            }
        ]
    }"""

print update(req)
