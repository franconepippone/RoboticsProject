from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf2_geometry_msgs

# NB make sure to append this line "${workspaceFolder}/install/custom_interfaces/lib/python3.12/site-packages" in .vscode/settings.json under
# the "python.analysis.extraPaths" key, otherwise vscode might fail to properly provide type hints. Of course, you must have built the ws for this to work.
from custom_interfaces.srv import GetTagPosition
from geometry_msgs.msg import PointStamped


# Where april_tag_detector is publishing detections.
# NOTE that this topic is remapped in the launch file, to account for the namespace the april_tag_detector node lives in.
# By default, the resolved topic name is /apriltag_detection/tag_detection
INCOMING_TAG_DETECTION_TOPIC = '/tag_detection'

GLOBAL_FRAME_ID = "map" # ideally, this should be changed to "map" when using slam

class TagMapper(Node):
    """
    Ros2 Node for storing received tag position data from the apriltag_detector node,
    transforming points into global coordinates and remembering their position associated with the tag id. 
    
    A service under /<namespace>/get_tag_position is available for other nodes to get world relative
    coordinates of already found apriltags. By default, <namespace> is 'apriltag_detection' if you use the provided launch files.t this topic is remapped in the launch file, to account for the namespace the april_tag_detector node lives in.
# By default, the resolved topic name is /apriltag_detection/tag_detection
INCOMING_TAG_DETECTION_TOPIC = '/tag_detection'

GLOBAL_FRAME_ID = "odom" # ideally, this should be changed to "map" when using slam
 
    """

    def __init__(self):
        super().__init__('tag_mapper')

        # Cache: tag_id -> PointStamped (map frame)
        self.tag_cache: dict[int, PointStamped] = {}

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self)

        # Subscriber
        self.sub = self.create_subscription(
            PointStamped,
            INCOMING_TAG_DETECTION_TOPIC,
            self.process_tag_callback,
            10
        )

        # Marker publisher
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'apriltag_markers',
            10
        )

        # expose service for other nodes to get tags positions
        self.get_tag_srv = self.create_service(
            GetTagPosition,
            'get_tag_position',
            self.get_tag_position_callback
        )

        # Timer to republish markers
        self.create_timer(0.5, self.publish_markers)
        self.get_logger().info("April tag mapper started")

    def get_tag_position_callback(
        self, 
        request: GetTagPosition.Request, 
        response: GetTagPosition.Response
    ) -> GetTagPosition.Response:
        """
        Service callback to return the position of a tag in map frame.

        Args:
            request (GetTagPosition.Request): The request containing tag_id
            response (GetTagPosition.Response): The response object to populate

        Returns:
            GetTagPosition.Response: Populated response with found flag and position
        """
        tag_id: int = request.tag_id
        cached_point: Optional[PointStamped] = self.tag_cache.get(tag_id)

        if cached_point is not None:
            response.found = True
            response.position = cached_point
        else:
            response.found = False
            response.position = PointStamped()  # empty PointStamped

        return response


    def extract_tag_id(self, msg: PointStamped) -> tuple[str, int]:
        """
        Example frame_id: 'camera_link:=:7'
        Returns original frame id and decoded tag
        """
        frame = msg.header.frame_id
        if ':=:' in frame:
            frame_id_real, tag_id = frame.split(':=:')
            return frame_id_real, int(tag_id)
        else:
            return frame, -1  # unknown

    def process_tag_callback(self, point: PointStamped):
        """
        Converts from camera_frame coordinates to world (map or odom) coordinates,
        populates local tags cache with tag id and its respective world position,
        so that it can later be retrieved with 'get_tag_position' service.
        
        :param point: Point expressed in camera link frame coordinates where the tag was found
        :type point: PointStamped
        """
        frame_id, tag_id = self.extract_tag_id(point)
        if tag_id < 0:
            # tag_id is -1 if extraction failed
            self.get_logger().warn("Could not extract tag ID")
            return

        try:
            # Lookup transform: camera_frame -> map
            transform = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME_ID,                    # target frame
                frame_id,      # source frame
                Time()
            )

            # Transform the point
            point_in_map = tf2_geometry_msgs.do_transform_point(
                point,
                transform
            )

            # Store in cache
            self.tag_cache[tag_id] = point_in_map
            #self.tag_cache[tag_id] = point  # temporarly store raw point

            self.get_logger().info(
                f"Stored tag {tag_id} at "
                f"({point_in_map.point.x:.2f}, "
                f"{point_in_map.point.y:.2f}, "
                f"{point_in_map.point.z:.2f}) in '{GLOBAL_FRAME_ID}' frame"
                f"(original xyz: {point.point.x:.2f} {point.point.y:.2f} {point.point.z:.2f})"
            )

        except Exception as e:
            self.get_logger().warn(
                f"TF transform failed for tag {tag_id}: {e}"
            )

    def publish_markers(self):
        marker_array = MarkerArray()

        for tag_id, point in self.tag_cache.items():

            marker = Marker()
            marker.header.frame_id = GLOBAL_FRAME_ID #'oakd_link' #GLOBAL_FRAME_ID
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = 'apriltags'
            marker.id = tag_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position = point.point
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15

            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 1.0

            marker_array.markers.append(marker) # type: ignore

        self.marker_pub.publish(marker_array)


# ----------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------

def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TagMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
