import sys
import threading

import rclpy
from rclpy.node import Node

from mocha_buono.sensor.SensorService import SensorService
from mocha_buono.sensor.SensorParameters import SensorParameters
from mocha_buono.sensor.SensorErrors import BusOverRunException



class Mocha(Node):
    sensor = None
    param = None

    def __init__(self):
        super().__init__('mocha_buono')

    def setup(self):
        self.param = SensorParameters(self)
        if self.param.frame_id.value == "mocha_buono":
            pass
        else:
            pass
        self.sensor = SensorService(self, self.param)
        self.sensor.configure()


def main(args=None):
    try:
        rclpy.init()

        # Create & initialize ROS2 node:
        node = Mocha()
        node.setup()

        # Create lock object to prevent overlapping data queries
        lock = threading.Lock()

        def read_data():
            """Periodic data_query_timer executions to retrieve sensor IMU data."""
            if lock.locked():
                # critical area still locked that means that the previous data query is still being processed
                node.get_logger().warn('Message communication in progress - skipping query cycle')
                return

            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            try:
                # perform synchronized block:
                node.sensor.get_sensor_data()
            except BusOverRunException:
                # data not available yet, wait for next cycle | see #5
                return
            except ZeroDivisionError:
                # division by zero in get_sensor_data, return
                return
            except Exception as e:  # noqa: B902
                node.get_logger().warn('Receiving sensor data failed with %s:"%s"' % (type(e).__name__, e))
            finally:
                lock.release()

        def log_calibration_status():
            """Periodic logging of calibration data (quality indicators)."""
            if lock.locked():
                # critical area still locked
                # that means that the previous data query is still being processed
                node.get_logger().warn('Message communication in progress - skipping query cycle')
                # traceback.print_exc()
                return

            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            try:
                # perform synchronized block:
                node.sensor.get_calib_status()
            except Exception as e:  # noqa: B902
                node.get_logger().warn('Receiving calibration status failed with %s:"%s"' % (type(e).__name__, e))
                # traceback.print_exc()
            finally:
                lock.release()

        # start regular sensor transmissions - frequencies around 30Hz and above might cause performance impacts:
        f = 1.0 / float(node.param.data_query_frequency.value)
        data_query_timer = node.create_timer(f, read_data)

        # start regular calibration status logging
        f = 1.0 / float(node.param.calib_status_frequency.value)
        status_timer = node.create_timer(f, log_calibration_status)

        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received - exiting...')
        sys.exit(0)
    finally:
        node.get_logger().info('ROS node shutdown')
        try:
            node.destroy_timer(data_query_timer)
            node.destroy_timer(status_timer)
        except UnboundLocalError:
            node.get_logger().info('No timers to shutdown')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()