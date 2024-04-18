import sys
import time
import numpy as np
import rclpy
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('spawn_entity')
        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.spawn_req = SpawnEntity.Request()

        self.del_client = self.create_client(DeleteEntity, 'delete_entity')
        while not self.del_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.del_req = DeleteEntity.Request()

    def send_request(self):

        object_names = ["A", "B", "C"]
        object_colors = [
            "1.0 0.1 0.1 1.0",
            "0.0 1.0 0.0 1.0",
            "0.0 0.0 1.0 1.0"
        ]

        for (name, color) in zip(object_names, object_colors):

            self.del_req.name = name
            self.future = self.del_client.call_async(self.del_req)

            time.sleep(1)

            self.spawn_req.name = name
            self.spawn_req.xml = "<?xml version=\"1.0\" ?><sdf version=\"1.5\"><model name=\"will_be_ignored\"><static>true</static><link name=\"link\"><visual name=\"visual\"><geometry><sphere><radius>0.04</radius></sphere></geometry><material><ambient rgba=\"{0:}\"/><diffuse rgba=\"{0:}\"/><specular rgba=\"0.2 0.2 0.2 1.0\"/><emissive>{0:}</emissive></material></visual></link></model></sdf>".format(color)
            self.spawn_req.initial_pose.position.x = np.random.rand() * 1.1
            self.spawn_req.initial_pose.position.y = np.random.rand() * 2. - 1.
            self.spawn_req.initial_pose.position.z = 0.5 + (np.random.rand() * 0.5)
            self.future = self.spawn_client.call_async(self.spawn_req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Test {}'.format(minimal_client.spawn_req.xml))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()