#!/usr/bin/env python3
import math

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry


class BatteryNode(Node):
    """Simulated battery.

    Publishes sensor_msgs/BatteryState on /battery_state at `publish_rate` Hz (node clock, i.e. sim time).
    `percentage` is in [0, 1]. The level drains at `drain_rate` [1/s] and, while charging,
    rises at `charge_rate` [1/s].

    The battery is charging when the robot (base_link) is within `dock_radius` metres of the
    dock station center (`dock_x`, `dock_y` in the map frame). If dock_x or dock_y is NaN the
    proximity detection is disabled. Charging can also be forced on with /set_charging (std_srvs/SetBool) for debugging.

    While charging, `current` is +`charge_current` [A]; otherwise -`drain_current` [A]. Nav2's
    SimpleChargingDock detects the end of the docking from this value (REP 117 sign convention).
    """

    def __init__(self):
        super().__init__('battery_node')

        self.declare_parameter('publish_rate', 5.0)      # Hz: how often level is updated and published
        self.declare_parameter('initial_level', 1.0)     # [0, 1]
        self.declare_parameter('drain_rate', 0.002)      # 1/s while discharging
        self.declare_parameter('charge_rate', 0.02)      # 1/s while charging
        self.declare_parameter('charge_current', 2.0)    # A reported while charging
        self.declare_parameter('drain_current', 1.0)     # A reported (as negative) while discharging
        self.declare_parameter('dock_x', math.nan)       # dock center in map frame (NaN: disabled)
        self.declare_parameter('dock_y', math.nan)
        self.declare_parameter('dock_radius', 0.15)      # m
        self.declare_parameter('dock_max_speed', 0.05)   # m/s: faster than this = just passing over
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')

        publish_rate = float(self.get_parameter('publish_rate').value)
        self.drain_rate = float(self.get_parameter('drain_rate').value)
        self.charge_rate = float(self.get_parameter('charge_rate').value)
        self.charge_current = float(self.get_parameter('charge_current').value)
        self.drain_current = float(self.get_parameter('drain_current').value)
        self.dock_x = float(self.get_parameter('dock_x').value)
        self.dock_y = float(self.get_parameter('dock_y').value)
        self.dock_radius = float(self.get_parameter('dock_radius').value)
        self.dock_max_speed = float(self.get_parameter('dock_max_speed').value)
        self.speed = math.inf     # unknown until first odometry message
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        if publish_rate <= 0.0: raise ValueError("'publish_rate' must be positive")
        if self.drain_rate < 0.0 or self.charge_rate < 0.0: raise ValueError("'drain_rate' and 'charge_rate' must be non-negative")

        self.level = min(1.0, max(0.0, float(self.get_parameter('initial_level').value)))
        self.forced_charging = False    # set by /set_charging
        self.on_dock = False            # robot is on the dock station
        self._charging = False          # last published state (to log changes)
        self._last = None               # time of the previous update (None until first tick)

        self.dock_enabled = not (math.isnan(self.dock_x) or math.isnan(self.dock_y))
        if self.dock_enabled:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        else:
            self.get_logger().warning("dock_x/dock_y not set: charging only via /set_charging")

        self.pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.create_service(SetBool, '/set_charging', self.on_set_charging)
        self.create_timer(1.0 / publish_rate, self.on_timer)

        self.get_logger().info(
            f'Battery node started: level={self.level:.0%}, rate={publish_rate} Hz, '
            f'drain={self.drain_rate}/s, charge={self.charge_rate}/s')

    def on_set_charging(self, request, response):
        self.forced_charging = request.data
        response.success = True
        response.message = 'charging forced' if request.data else 'charging not forced'
        return response

    def on_odom(self, msg):
        v = msg.twist.twist.linear
        self.speed = math.hypot(v.x, v.y) # compute |v|

    def _update_on_dock(self):
        """Robot is on the dock when base_link is within dock_radius of the dock center."""
        if not self.dock_enabled:
            self.on_dock = False
            return
        
        try: t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
        except tf2_ros.TransformException:
            self.on_dock = False      # localization not available yet
            return
        dist = math.hypot(t.transform.translation.x - self.dock_x,
                          t.transform.translation.y - self.dock_y)
        self.on_dock = dist <= self.dock_radius and self.speed <= self.dock_max_speed
        
    @property
    def charging(self) -> bool:
        return self.forced_charging or self.on_dock

    def on_timer(self):
        now = self.get_clock().now()
        self._update_on_dock()

        # Integrate level over the elapsed (simulated) time
        if self._last is not None:
            dt = (now - self._last).nanoseconds * 1e-9
            if dt > 0.0:     # skip if clock went backwards (sim reset)
                rate = self.charge_rate if self.charging else -self.drain_rate
                self.level = min(1.0, max(0.0, self.level + rate * dt))
        self._last = now

        if self.charging != self._charging:
            self._charging = self.charging
            self.get_logger().info(f'Charging {"started" if self._charging else "stopped"} ({self.level:.0%})')

        self.pub.publish(self._build_msg(now))

    def _build_msg(self, now) -> BatteryState:
        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.percentage = float(self.level)
        msg.present = True
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

        if self.charging:
            full = self.level >= 1.0
            msg.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_FULL if full
                                       else BatteryState.POWER_SUPPLY_STATUS_CHARGING)
            msg.current = self.charge_current
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            msg.current = -self.drain_current

        # Quantities that are not simulated are reported as unknown (REP 117)
        nan = math.nan
        msg.voltage = msg.temperature = nan
        msg.charge = msg.capacity = msg.design_capacity = nan
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()