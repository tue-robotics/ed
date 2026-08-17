#!/usr/bin/env python3

import rclpy

from ed_interfaces.srv import UpdateSrv

rclpy.init()
node = rclpy.create_node('ed_reset_custom_properties')

client = node.create_client(UpdateSrv, '/ed/update')
client.wait_for_service()

req = UpdateSrv.Request()
req.request = """{
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

future = client.call_async(req)
rclpy.spin_until_future_complete(node, future)
print(future.result().response)

rclpy.shutdown()
