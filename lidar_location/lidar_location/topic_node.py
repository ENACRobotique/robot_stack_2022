from lidar_location.Triangulation import Triangulation
from lidar_location.Object_list import Object_list
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class lidarlocation(Node):
    def __init__(self, name='Lidar_location'):
        super().__init__(name)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription
        self.publisher_ = self.create_publisher(LaserScan, 'filtered_scan', 10)

    def generate_filtered_message(self, message, filtered_data):
        out = message
        for i in range(0, len(filtered_data)):
            out.ranges[i] = filtered_data[i]
        return out

    def filter_out(self, message):
        out = []
        for i in range(0, len(message.ranges)):
            if message.ranges[i] is not None:
                if message.ranges[i] > 3.6:
                    out.append(0)
                else:
                    out.append(message.ranges[i])
            else:
                out.append(message.ranges[i])
        return out

    def echo(self, message):
        return message

    def listener_callback(self, msg):
        # self.get_logger().info(msg.angle_max)
        msg_out = self.generate_filtered_message(msg, self.filter_out(msg))
        #msg_out = msg
        # self.get_logger().info(msg_out.angle_max)
        #triangulation = Object_list(msg_out)
        triangulation = Triangulation(msg_out)
        self.get_logger().info("-------------")
        # for obj in triangulation.list_obj:
        #    self.get_logger().info(
        #        f"{obj.relative_center}")
        # self.publisher_.publish(msg_out)

        for tri in triangulation.tri_list:
            self.get_logger().info("Un triangle:")
            for pt in tri.pt_list:
                self.get_logger().info("Un point:")
                self.get_logger().info(
                    f"{pt.distance}")
                self.get_logger().info(
                    f"{pt.angle}")
        self.publisher_.publish(msg_out)


def main():
    rclpy.init()
    node = lidarlocation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
