import sys
import time
import numpy as np
import rclpy
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetModelList
from geometry_msgs.msg import Pose

from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('spawn_entity')

        self.models_client = self.create_client(GetModelList, 'get_model_list')
        while not self.models_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get Models service not available, waiting again...')
        self.models_req = GetModelList.Request()
        self.get_logger().info('Get Models service available.')

        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting again...')
        self.get_logger().info('Spawn service available.')

        self.del_client = self.create_client(DeleteEntity, 'delete_entity')
        while not self.del_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Delete service not available, waiting again...')
        self.get_logger().info('Delete service available.')

        self._publishers = [
            self.create_publisher(Pose, '/A_pose', 10),
            self.create_publisher(Pose, '/B_pose', 10),
            self.create_publisher(Pose, '/C_pose', 10)
        ]

    def send_request(self):

        object_names = ["A", "B", "C"]
        object_colors = [
            "1.0 0.0 0.2 1.0",
            "0.1 0.6 0.1 1.0",
            "0.0 0.3 1.0 1.0"
        ]
        color_names = ["red", "green", "blue"]


        future = self.models_client.call_async(self.models_req)

        rclpy.spin_until_future_complete(self, future)
        # while rclpy.ok() and not future.done():
        print("{}".format(future.result()))
            # time.sleep(0.5)
        
        existing_models = [m for m in future.result().model_names if m in object_names]

        for (name, color, color_name, publisher) in zip(object_names, object_colors, color_names, self._publishers):

            # delete previous object
            if name in existing_models:
                del_req = DeleteEntity.Request()
                del_req.name = name
                future = self.del_client.call_async(del_req)
                rclpy.spin_until_future_complete(self, future)
                print("Deleted object {}".format(name))
                # while rclpy.ok() and not future.done():
                #     time.sleep(0.5)
                time.sleep(2)

            # create Pose object message
            pose = Pose()
            pose.position.x = 0.2 + np.random.rand() * 0.5
            pose.position.y = np.random.rand() * 1.5 - 0.75
            pose.position.z = 0.1 + (np.random.rand() * 1.5)

            # create gazebo spawn request
            spawn_req = SpawnEntity.Request()
            spawn_req.name = name
            spawn_req.xml = " \
                <?xml version=\"1.0\" ?><sdf version=\"1.5\"> \
                <model name=\"will_be_ignored\"><static>true</static> \
                <link name=\"link\"> \
                <visual name=\"visual\"> \
                    <geometry> \
                        <sphere><radius>0.04</radius></sphere> \
                    </geometry> \
                    <material> \
                        <ambient>{0:}</ambient> \
                        <diffuse>0.5 0.5 0.5 1</diffuse> \
                        <specular>1 1 1 1</specular> \
                        <emissive>{0:}</emissive> \
                    </material> \
                </visual> \
                </link></model></sdf>".format(color)
            spawn_req.initial_pose.position.x = pose.position.x
            spawn_req.initial_pose.position.y = pose.position.y
            spawn_req.initial_pose.position.z = pose.position.z

            # spawn new object
            future = self.spawn_client.call_async(spawn_req)
            rclpy.spin_until_future_complete(self, future)

            # publish on topic
            publisher.publish(pose)
            
            self.get_logger().info("{}: {}, ({:.3f}, {:.3f}, {:.3f})".format(
                name, 
                color_name, 
                pose.position.x,
                pose.position.y,
                pose.position.z
            ))



def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    rclpy.spin_once(minimal_client)
    rclpy.spin_once(minimal_client)

    # while rclpy.ok():
    # rclpy.spin_once(minimal_client)
        # if minimal_client.future.done():
        #     try:
        #         response = minimal_client.future.result()
        #     except Exception as e:
        #         minimal_client.get_logger().info(
        #             'Service call failed %r' % (e,))
        #     else:
        #         minimal_client.get_logger().info(
        #             'Done')
        #     break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()