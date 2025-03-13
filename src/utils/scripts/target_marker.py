#!/usr/bin/env python3


# TODO make this as a package and launch it in the ROS way. that might solve the issue?


import rclpy
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped, TransformStamped
from builtin_interfaces.msg import Time
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
import time
import numpy as np

class TargetMarkerNode(Node):
    def __init__(self):
        super().__init__('target_marker_node')
        

        time.sleep(2)
        # Create an Interactive Marker Server
        self.server = InteractiveMarkerServer(self, 'target_marker')
        
        time.sleep(2)

        # Create publisher for target pose
        self.pose_publisher = self.create_publisher(PoseStamped, 'target_pose', 10)



        self.tf_broadcaster = StaticTransformBroadcaster(self)

        
        # Create the interactive marker
        self.create_target_marker()
        self.get_logger().info("Target marker initialized. Add 'Interactive Markers' display in RViz and set the Update Topic to /target_marker/update")
        
    def create_target_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base"  
        int_marker.name = "target_marker"
        int_marker.scale = 0.07
        int_marker.pose.position.x = 0.2
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.243
        # The orientation is set to align the marker with the z-axis
        int_marker.pose.orientation.x = 0.0
        int_marker.pose.orientation.y = 0.0
        int_marker.pose.orientation.z = 0.0
        int_marker.pose.orientation.w = 1.0
        int_marker.description = "Target Pose Control"
        
        # Create a visible sphere as the marker
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Create a control with the visible marker
        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.name = int_marker.name
        visual_control.interaction_mode = InteractiveMarkerControl.NONE
        # visual_control.markers.append(marker)
        int_marker.controls.append(visual_control)
        
        # Add a move control with 3D arrows
        control = InteractiveMarkerControl()
        control.name = int_marker.name
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        int_marker.controls.append(control)
        
        # Add a rotate control with 3D arrows
        control = InteractiveMarkerControl()
        control.name = int_marker.name
        control.interaction_mode = InteractiveMarkerControl.ROTATE_3D
        int_marker.controls.append(control)
        
        # Create 6-DOF controls
        # Rotation controls
        for axis, orientation in [
            ('x', (1., 0., 0., 1.)),  # (x, y, z, w)
            ('y', (0., 1., 0., 1.)),
            ('z', (0., 0., 1., 1.))
        ]:
            # Rotation control
            control = InteractiveMarkerControl()
            control.orientation.x = orientation[0]
            control.orientation.y = orientation[1]
            control.orientation.z = orientation[2]
            control.orientation.w = orientation[3]
            control.name = f"rotate_{axis}"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(control)
            
            # Translation control
            control = InteractiveMarkerControl()
            control.orientation.x = orientation[0]
            control.orientation.y = orientation[1]
            control.orientation.z = orientation[2]
            control.orientation.w = orientation[3]
            control.name = f"move_{axis}"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control)
        
        # Add the interactive marker to the server
        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.server.setCallback(int_marker.name, self.process_feedback)

        # # Set the pose
        # pose = PoseStamped()
        # pose.pose.position.x = 0.2
        # pose.pose.position.y = 0.0
        # pose.pose.position.z = 0.243
        # pose.pose.orientation.x = 0.50
        # pose.pose.orientation.y = -0.50
        # pose.pose.orientation.z = 0.50
        # pose.pose.orientation.w = -0.50
        # int_marker.pose = pose.pose

        # self.server.setPose(int_marker.name, pose.pose)

        self.server.applyChanges()
        self.get_logger().info(f"Interactive marker created with name: {int_marker.name}")

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = int_marker.header.frame_id
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose = int_marker.pose
        self.pose_publisher.publish(pose_msg)

        # add frame to tf
        th = TransformStamped()
        th.header.stamp = self.get_clock().now().to_msg()
        th.header.frame_id = 'base'
        th.child_frame_id = 'hidden'
        th.transform.translation.x = int_marker.pose.position.x
        th.transform.translation.y = int_marker.pose.position.y
        th.transform.translation.z = int_marker.pose.position.z
        th.transform.rotation.x = int_marker.pose.orientation.x
        th.transform.rotation.y = int_marker.pose.orientation.y
        th.transform.rotation.z = int_marker.pose.orientation.z
        th.transform.rotation.w = int_marker.pose.orientation.w

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.stamp.nanosec += 3
        t.header.frame_id = 'hidden'
        t.child_frame_id = 'target'
        # t.transform.translation = th.transform.translation
        # q = quaternion_from_euler(0, np.pi/2, 0)
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(th)
        self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(f"Publishing target pose: position=({pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f}, {pose_msg.pose.position.z:.2f})")


    def process_feedback(self, feedback):
        # Create and publish PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = feedback.header.frame_id
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose = feedback.pose
        self.pose_publisher.publish(pose_msg)

        # add frame to tf
        th = TransformStamped()
        th.header.stamp = self.get_clock().now().to_msg()
        th.header.frame_id = 'base'
        th.child_frame_id = 'hidden'
        th.transform.translation.x = feedback.pose.position.x
        th.transform.translation.y = feedback.pose.position.y
        th.transform.translation.z = feedback.pose.position.z
        th.transform.rotation.x = feedback.pose.orientation.x
        th.transform.rotation.y = feedback.pose.orientation.y
        th.transform.rotation.z = feedback.pose.orientation.z
        th.transform.rotation.w = feedback.pose.orientation.w

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.stamp.nanosec += 3
        t.header.frame_id = 'hidden'
        t.child_frame_id = 'target'
        # t.transform.translation = th.transform.translation
        # q = quaternion_from_euler(0, np.pi/2, 0)
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(th)
        self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(f"Publishing target pose: position=({pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f}, {pose_msg.pose.position.z:.2f})")

def main():
    rclpy.init()
    node = TargetMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()