#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy.duration
from geometry_msgs.msg import Point

class GTLoggerNode(Node):
    def __init__(self):
        super().__init__('gt_tf_logger')

        # Parameters
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('output_file', '/home/hesam/ros2_test/outputs/trajectories/gt_trajectory.tum')
        self.declare_parameter('publish_static_tf', False)
        self.declare_parameter('publish_odometry', True)

        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.publish_static_tf = self.get_parameter('publish_static_tf').get_parameter_value().bool_value
        self.publish_odometry = self.get_parameter('publish_odometry').get_parameter_value().bool_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, '/gt/gps_no_drift', 10)

        self.trajectory = []
        self.timer = self.create_timer(0.05, self.poll_tf)

        self.get_logger().info(f"Recording TF from '{self.parent_frame}' → '{self.child_frame}'")

    def poll_tf(self):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.2)
            )

            stamp = trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9
            t = trans.transform.translation
            q = trans.transform.rotation

            self.trajectory.append((stamp, t.x, t.y, t.z, q.x, q.y, q.z, q.w))

            if self.publish_static_tf:
                static_tf = TransformStamped()
                static_tf.header.stamp = self.get_clock().now().to_msg()
                static_tf.header.frame_id = self.parent_frame
                static_tf.child_frame_id = f"{self.child_frame}_gt"
                static_tf.transform.translation = t
                static_tf.transform.rotation = q
                self.static_broadcaster.sendTransform(static_tf)

            if self.publish_odometry:
                odom_msg = Odometry()
                odom_msg.header.stamp = trans.header.stamp
                odom_msg.header.frame_id = self.parent_frame
                odom_msg.child_frame_id = self.child_frame

                odom_msg.pose.pose.position = Point(x=t.x, y=t.y, z=t.z)
                odom_msg.pose.pose.orientation = q

                self.odom_pub.publish(odom_msg)

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def destroy_node(self):
        try:
            with open(self.output_file, 'w') as f:
                for entry in self.trajectory:
                    line = "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(*entry)
                    f.write(line)
            self.get_logger().info(f"TUM trajectory saved to {self.output_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save TUM trajectory: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GTLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
