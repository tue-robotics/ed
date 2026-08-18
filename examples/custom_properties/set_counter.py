#!/usr/bin/env python3

import rclpy

from ed_interfaces.srv import UpdateSrv

rclpy.init()
node = rclpy.create_node('ed_set_counter')

client = node.create_client(UpdateSrv, '/ed/update')
client.wait_for_service()

req = UpdateSrv.Request()
req.request = '{"entities": [{"id":"test-entity", "properties":[{"name":"counter", "value": 1000}]}]}'

future = client.call_async(req)
rclpy.spin_until_future_complete(node, future)
print(future.result().response)

rclpy.shutdown()
