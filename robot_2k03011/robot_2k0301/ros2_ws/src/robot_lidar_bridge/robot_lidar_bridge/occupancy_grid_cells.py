import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells, OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class OccupancyGridCells(Node):
    def __init__(self):
        super().__init__('occupancy_grid_cells')
        self.declare_parameter('occupied_threshold', 65)
        self.occupied_threshold = max(
            0, min(100, int(self.get_parameter('occupied_threshold').value)))

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            GridCells, 'map_occupied_cells', map_qos)
        self.subscription = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, map_qos)
        self.last_shape = None

    def map_callback(self, message):
        resolution = message.info.resolution
        width = message.info.width
        height = message.info.height
        if resolution <= 0.0 or width == 0 or height == 0:
            self.get_logger().warning('ignored invalid occupancy grid')
            return

        orientation = message.info.origin.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y)
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(sin_yaw, cos_yaw)
        cos_origin = math.cos(yaw)
        sin_origin = math.sin(yaw)
        origin_x = message.info.origin.position.x
        origin_y = message.info.origin.position.y

        output = GridCells()
        output.header = message.header
        output.cell_width = resolution
        output.cell_height = resolution
        for index, occupancy in enumerate(message.data):
            if occupancy < self.occupied_threshold:
                continue
            column = index % width
            row = index // width
            local_x = (column + 0.5) * resolution
            local_y = (row + 0.5) * resolution
            point = Point()
            point.x = origin_x + cos_origin * local_x - sin_origin * local_y
            point.y = origin_y + sin_origin * local_x + cos_origin * local_y
            point.z = 0.0
            output.cells.append(point)

        self.publisher.publish(output)
        shape = (width, height, resolution, len(output.cells))
        if shape != self.last_shape:
            self.get_logger().info(
                f'map geometry ready: {width}x{height} '
                f'resolution={resolution:.3f} occupied_cells={len(output.cells)}')
            self.last_shape = shape


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridCells()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
