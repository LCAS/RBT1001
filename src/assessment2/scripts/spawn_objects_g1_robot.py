import sys
import time
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('spawn_entity')


        self.tf_broadcaster = StaticTransformBroadcaster(self)


        self.marker_array_pub = self.create_publisher(MarkerArray, "/task_markers", 10)

        self._publishers = [
            self.create_publisher(Pose, '/A_pose', 10),
            self.create_publisher(Pose, '/B_pose', 10),
            self.create_publisher(Pose, '/C_pose', 10)
        ]

        self.transforms = []
        self.send_request()

        period = 0.1
        self.timer = self.create_timer(period, self.pub_transform)

    def send_request(self):

        object_names = ["A", "B", "C"]
        object_colors = [
            [1.0, 0.0, 0.2, 1.0],
            [0.1, 0.6, 0.1, 1.0],
            [0.0, 0.3, 1.0, 1.0]
        ]
        color_names = ["red", "green", "blue"]

        marker_array_msg = MarkerArray()

        for i, (name, color, color_name, publisher) in enumerate(zip(object_names, object_colors, color_names, self._publishers)):
            # create Pose object message
            pose = Pose()
            if name == "A":
                pose.position.x = 0.5
                pose.position.y = -0.5
                pose.position.z = 0.85
            elif name == "C":
                pose.position.x = 0.5
                pose.position.y = 0.5
                pose.position.z = 0.85
            else:
                pose.position.x = 0.6 + np.random.rand() * 0.3
                pose.position.y = np.random.rand() * 1 - 0.5
                pose.position.z = 0.75 + (np.random.rand() * 0.5)

            # add frame
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = name
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.transforms.append(t)

            # add visual marker for rviz
            marker_msg = Marker()
            marker_msg.ns = name
            marker_msg.id = i
            marker_msg.type = Marker.SPHERE
            marker_msg.header.frame_id = "base_link"
            marker_msg.scale.x = 5e-2
            marker_msg.scale.y = 5e-2
            marker_msg.scale.z = 5e-2
            marker_msg.color.r = color[0]
            marker_msg.color.g = color[1]
            marker_msg.color.b = color[2]
            marker_msg.color.a = 1.0
            marker_msg.pose.position.x = pose.position.x
            marker_msg.pose.position.y = pose.position.y
            marker_msg.pose.position.z = pose.position.z
            marker_msg.pose.orientation.w = 1.
            marker_array_msg.markers.append(marker_msg)

            # publish on topic
            publisher.publish(pose)
            
            self.get_logger().info("{}: {}, ({:.3f}, {:.3f}, {:.3f})".format(
                name, 
                color_name, 
                pose.position.x,
                pose.position.y,
                pose.position.z
            ))

        # publish list of rviz markers
        self.marker_array_pub.publish(marker_array_msg)

    def pub_transform(self):
        for t in self.transforms:
            self.tf_broadcaster.sendTransform(t)



def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()

    rclpy.spin(minimal_client)
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