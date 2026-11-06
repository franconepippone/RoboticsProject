import rclpy

from rclpy.node import Node
from typing import cast

from std_srvs.srv import Trigger

from pallet_pickup.dock_pose_bridge import DockPoseBridge


class DockPoseBridgeNode(Node):
    """ Wrapper for the DockPoseBridge component to run it as a standalone node. 
    Configurable via ROS parameters and provides start/stop services. """

    def __init__(self):
        super().__init__('dock_pose_bridge')

        # Parameters
        self.declare_parameter('tag_id', 0)
        self.declare_parameter('tag_frame_prefix', 'tag36h11:')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('output_topic', '/detected_dock_pose')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('max_detection_age', 0.5)
        self.declare_parameter('auto_start', False)

        tag_id: int = self.get_parameter('tag_id').get_parameter_value().integer_value
        tag_frame_prefix: str = self.get_parameter('tag_frame_prefix').get_parameter_value().string_value
        target_frame: str = self.get_parameter('target_frame').get_parameter_value().string_value
        output_topic: str = self.get_parameter('output_topic').get_parameter_value().string_value
        publish_rate: float = self.get_parameter('publish_rate').get_parameter_value().double_value
        max_detection_age: float = self.get_parameter('max_detection_age').get_parameter_value().double_value
        auto_start: bool = self.get_parameter('auto_start').get_parameter_value().bool_value

        # bridge component
        self.bridge = DockPoseBridge(
            node=self,
            target_frame=target_frame,
            tag_frame_prefix=tag_frame_prefix,
            output_topic=output_topic,
            publish_rate=publish_rate,
            max_detection_age=max_detection_age,
        )

        self.tag_id: int = tag_id

        # Services
        self.start_service = self.create_service(
            Trigger,
            '~/start',
            self.start_callback,
        )

        self.stop_service = self.create_service(
            Trigger,
            '~/stop',
            self.stop_callback,
        )

        self.get_logger().info(
            f'Dock pose bridge ready. Target tag: {self.tag_id}'
        )

        if auto_start:
            self.bridge.start(self.tag_id)
            self.get_logger().info(
                f'Auto-start enabled: bridge active for tag {self.tag_id}'
            )

    def start_callback(self, request, response):
        self.bridge.start(self.tag_id)

        response.success = True
        response.message = f'Bridge started for tag {self.tag_id}'
        return response

    def stop_callback(self, request, response):
        self.bridge.stop()

        response.success = True
        response.message = 'Bridge stopped'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DockPoseBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bridge.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()