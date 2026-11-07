#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool


class BatteryNode(Node):
    """Simulated battery.

    Publishes sensor_msgs/BatteryState on /battery_state at `publish_rate` Hz (node clock, i.e. sim time).
    `percentage` is in [0, 1]. The level drains at `drain_rate` [1/s] and, while charging,
    rises at `charge_rate` [1/s]. Charging is switched on/off through the /set_charging service (std_srvs/SetBool).
    """

    def __init__(self):
        super().__init__('battery_node')

        self.declare_parameter('publish_rate', 2.0)      # Hz: how often level is updated and published
        self.declare_parameter('initial_level', 1.0)     # [0, 1]
        self.declare_parameter('drain_rate', 0.002)      # 1/s while discharging
        self.declare_parameter('charge_rate', 0.02)      # 1/s while charging

        publish_rate = float(self.get_parameter('publish_rate').value)
        self.drain_rate = float(self.get_parameter('drain_rate').value)
        self.charge_rate = float(self.get_parameter('charge_rate').value)

        if publish_rate <= 0.0: raise ValueError("'publish_rate' must be positive")
        if self.drain_rate < 0.0 or self.charge_rate < 0.0: raise ValueError("'drain_rate' and 'charge_rate' must be non-negative")

        self.level = min(1.0, max(0.0, float(self.get_parameter('initial_level').value)))
        self.charging = False
        self._last = None     # time of the previous update (None until first tick)

        self.pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.create_service(SetBool, '/set_charging', self.on_set_charging)
        self.create_timer(1.0 / publish_rate, self.on_timer)

        self.get_logger().info(
            f'Battery node started: level={self.level:.0%}, rate={publish_rate} Hz, '
            f'drain={self.drain_rate}/s, charge={self.charge_rate}/s')

    def on_set_charging(self, request, response):
        if request.data != self.charging:
            self.charging = request.data
            self.get_logger().info(f'Charging {"started" if self.charging else "stopped"} ({self.level:.0%})')
        response.success = True
        response.message = 'charging' if self.charging else 'not charging'
        return response

    def on_timer(self):
        now = self.get_clock().now()

        # Integrate level over the elapsed (simulated) time
        if self._last is not None:
            dt = (now - self._last).nanoseconds * 1e-9
            if dt > 0.0:     # skip if clock went backwards (sim reset)
                rate = self.charge_rate if self.charging else -self.drain_rate
                self.level = min(1.0, max(0.0, self.level + rate * dt))
                
        self._last = now
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
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        # Quantities that are not simulated are reported as unknown (REP 117)
        nan = math.nan
        msg.voltage = msg.temperature = msg.current = nan
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