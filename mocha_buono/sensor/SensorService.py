import json
from math import sqrt
import struct
import sys
from time import sleep

import board
import adafruit_bno055

from mocha_buono.sensor.SensorParameters import SensorParameters

from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String
from example_interfaces.srv import Trigger


i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

class SensorService:
    """Provide an interface for accessing the sensor's features & data."""

    def __init__(self, node: Node, param: SensorParameters):
        self.node = node
        self.param = param
        self.board = adafruit_bno055.BNO055_I2C(board.I2C())
        prefix = self.param.ros_topic_prefix.value
        QoSProf = QoSProfile(depth=10)

        # create topic publishers:
        self.pub_imu_raw = node.create_publisher(Imu, prefix + 'imu_raw', QoSProf)
        self.pub_imu = node.create_publisher(Imu, prefix + 'imu', QoSProf)
        self.pub_mag = node.create_publisher(MagneticField, prefix + 'mag', QoSProf)
        self.pub_temp = node.create_publisher(Temperature, prefix + 'temp', QoSProf)
        self.pub_calib_status = node.create_publisher(String, prefix + 'calib_status', QoSProf)
        self.srv = self.node.create_service(Trigger, prefix + 'calibration_request', self.calibration_request_callback)

    def configure(self):
        """Configure the IMU sensor hardware."""
        self.node.get_logger().info('Configuring device...')
        # try:
        #     data = self.con.receive(registers.BNO055_CHIP_ID_ADDR, 1)
        #     if data[0] != registers.BNO055_ID:
        #         raise IOError('Device ID=%s is incorrect' % data)
        #     # print("device sent ", binascii.hexlify(data))
        # except Exception as e:  # noqa: B902 
        #     # This is the first communication - exit if it does not work
        #     self.node.get_logger().error('Communication error: %s' % e)
        #     self.node.get_logger().error('Shutting down ROS node...')
        #     sys.exit(1)

        self.node.get_logger().info('Bosch BNO055 IMU configuration complete.')

    def get_sensor_data(self):
        """Read IMU data from the sensor, parse and publish."""
        # Initialize ROS msgs
        imu_raw_msg = Imu()
        imu_msg = Imu()
        mag_msg = MagneticField()
        temp_msg = Temperature()

        acceleration = self.board.acceleration
        gyro = self.board.gyro
        quaternion = self.board.quaternion
        linear_acceleration = self.board.linear_acceleration
        magnetic = self.board.magnetic
        temperature = self.board.temperature

        # read from sensor
        # buf = self.con.receive(registers.BNO055_ACCEL_DATA_X_LSB_ADDR, 45)
        # Publish raw data
        imu_raw_msg.header.stamp = self.node.get_clock().now().to_msg()
        imu_raw_msg.header.frame_id = self.param.frame_id.value
        # TODO: do headers need sequence counters now?
        # imu_raw_msg.header.seq = seq

        # TODO: make this an option to publish?
        imu_raw_msg.orientation_covariance = [
            self.param.variance_orientation.value[0], 0.0, 0.0,
            0.0, self.param.variance_orientation.value[1], 0.0,
            0.0, 0.0, self.param.variance_orientation.value[2]
        ]

        imu_raw_msg.linear_acceleration.x = acceleration[0] / self.param.acc_factor.value
        imu_raw_msg.linear_acceleration.y = acceleration[1] / self.param.acc_factor.value
        imu_raw_msg.linear_acceleration.z = acceleration[2] / self.param.acc_factor.value
        imu_raw_msg.linear_acceleration_covariance = [
            self.param.variance_acc.value[0], 0.0, 0.0,
            0.0, self.param.variance_acc.value[1], 0.0,
            0.0, 0.0, self.param.variance_acc.value[2]
        ]
        imu_raw_msg.angular_velocity.x = gyro[0] / self.param.gyr_factor.value
        imu_raw_msg.angular_velocity.y = gyro[1] / self.param.gyr_factor.value
        imu_raw_msg.angular_velocity.z = gyro[2] / self.param.gyr_factor.value
        imu_raw_msg.angular_velocity_covariance = [
            self.param.variance_angular_vel.value[0], 0.0, 0.0,
            0.0, self.param.variance_angular_vel.value[1], 0.0,
            0.0, 0.0, self.param.variance_angular_vel.value[2]
        ]
        # node.get_logger().info('Publishing imu message')
        self.pub_imu_raw.publish(imu_raw_msg)

        # TODO: make this an option to publish?
        # Publish filtered data
        imu_msg.header.stamp = self.node.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.param.frame_id.value

        q = Quaternion()
        # imu_msg.header.seq = seq
        q.w = quaternion[0]
        q.x = quaternion[1]
        q.y = quaternion[2]
        q.z = quaternion[3]
        # TODO(flynneva): replace with standard normalize() function
        # normalize
        norm = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        imu_msg.orientation.x = q.x / norm
        imu_msg.orientation.y = q.y / norm
        imu_msg.orientation.z = q.z / norm
        imu_msg.orientation.w = q.w / norm

        imu_msg.orientation_covariance = imu_raw_msg.orientation_covariance

        imu_msg.linear_acceleration.x = linear_acceleration[0] / self.param.acc_factor.value
        imu_msg.linear_acceleration.y = linear_acceleration[1]  / self.param.acc_factor.value
        imu_msg.linear_acceleration.z = linear_acceleration[2]  / self.param.acc_factor.value
        imu_msg.linear_acceleration_covariance = imu_raw_msg.linear_acceleration_covariance
        imu_msg.angular_velocity.x = gyro[0] / self.param.gyr_factor.value
        imu_msg.angular_velocity.y = gyro[1] / self.param.gyr_factor.value
        imu_msg.angular_velocity.z = gyro[2] / self.param.gyr_factor.value
        imu_msg.angular_velocity_covariance = imu_raw_msg.angular_velocity_covariance
        self.pub_imu.publish(imu_msg)

        # Publish magnetometer data
        mag_msg.header.stamp = self.node.get_clock().now().to_msg()
        mag_msg.header.frame_id = self.param.frame_id.value
        # mag_msg.header.seq = seq
        mag_msg.magnetic_field.x = magnetic[0] / self.param.mag_factor.value
        mag_msg.magnetic_field.y = magnetic[1]  / self.param.mag_factor.value
        mag_msg.magnetic_field.z = magnetic[2]/ self.param.mag_factor.value
        mag_msg.magnetic_field_covariance = [
            self.param.variance_mag.value[0], 0.0, 0.0,
            0.0, self.param.variance_mag.value[1], 0.0,
            0.0, 0.0, self.param.variance_mag.value[2]
        ]
        self.pub_mag.publish(mag_msg)

        # Publish temperature
        temp_msg.header.stamp = self.node.get_clock().now().to_msg()
        temp_msg.header.frame_id = self.param.frame_id.value
        # temp_msg.header.seq = seq
        temp_msg.temperature = float(temperature)
        self.pub_temp.publish(temp_msg)

    def get_calib_status(self):
        """
        Read calibration status for sys/gyro/acc/mag.

        Quality scale: 0 = bad, 3 = best
        """
        return

        calib_status = self.con.receive(registers.BNO055_CALIB_STAT_ADDR, 1)
        sys = (calib_status[0] >> 6) & 0x03
        gyro = (calib_status[0] >> 4) & 0x03
        accel = (calib_status[0] >> 2) & 0x03
        mag = calib_status[0] & 0x03

        # Create dictionary (map) and convert it to JSON string:
        calib_status_dict = {'sys': sys, 'gyro': gyro, 'accel': accel, 'mag': mag}
        calib_status_str = String()
        calib_status_str.data = json.dumps(calib_status_dict)

        # Publish via ROS topic:
        self.pub_calib_status.publish(calib_status_str)

    def get_calib_data(self):
        """Read all calibration data."""
        return

        accel_offset_read = self.con.receive(registers.ACCEL_OFFSET_X_LSB_ADDR, 6)
        accel_offset_read_x = (accel_offset_read[1] << 8) | accel_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        accel_offset_read_y = (accel_offset_read[3] << 8) | accel_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        accel_offset_read_z = (accel_offset_read[5] << 8) | accel_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        accel_radius_read = self.con.receive(registers.ACCEL_RADIUS_LSB_ADDR, 2)
        accel_radius_read_value = (accel_radius_read[1] << 8) | accel_radius_read[0]

        mag_offset_read = self.con.receive(registers.MAG_OFFSET_X_LSB_ADDR, 6)
        mag_offset_read_x = (mag_offset_read[1] << 8) | mag_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        mag_offset_read_y = (mag_offset_read[3] << 8) | mag_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        mag_offset_read_z = (mag_offset_read[5] << 8) | mag_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        mag_radius_read = self.con.receive(registers.MAG_RADIUS_LSB_ADDR, 2)
        mag_radius_read_value = (mag_radius_read[1] << 8) | mag_radius_read[0]

        gyro_offset_read = self.con.receive(registers.GYRO_OFFSET_X_LSB_ADDR, 6)
        gyro_offset_read_x = (gyro_offset_read[1] << 8) | gyro_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        gyro_offset_read_y = (gyro_offset_read[3] << 8) | gyro_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        gyro_offset_read_z = (gyro_offset_read[5] << 8) | gyro_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        calib_data = {'accel_offset': {'x': accel_offset_read_x, 'y': accel_offset_read_y, 'z': accel_offset_read_z}, 'accel_radius': accel_radius_read_value,
                      'mag_offset': {'x': mag_offset_read_x, 'y': mag_offset_read_y, 'z': mag_offset_read_z}, 'mag_radius': mag_radius_read_value,
                      'gyro_offset': {'x': gyro_offset_read_x, 'y': gyro_offset_read_y, 'z': gyro_offset_read_z}}

        return calib_data

    def print_calib_data(self):
        """Read all calibration data and print to screen."""

        return

        calib_data = self.get_calib_data()
        self.node.get_logger().info(
            '\tAccel offsets (x y z): %d %d %d' % (
                calib_data['accel_offset']['x'],
                calib_data['accel_offset']['y'],
                calib_data['accel_offset']['z']))

        self.node.get_logger().info(
            '\tAccel radius: %d' % (
                calib_data['accel_radius'],
            )
        )

        self.node.get_logger().info(
            '\tMag offsets (x y z): %d %d %d' % (
                calib_data['mag_offset']['x'],
                calib_data['mag_offset']['y'],
                calib_data['mag_offset']['z']))

        self.node.get_logger().info(
            '\tMag radius: %d' % (
                calib_data['mag_radius'],
            )
        )

        self.node.get_logger().info(
            '\tGyro offsets (x y z): %d %d %d' % (
                calib_data['gyro_offset']['x'],
                calib_data['gyro_offset']['y'],
                calib_data['gyro_offset']['z']))

    def set_calib_offsets(self, acc_offset, mag_offset, gyr_offset, mag_radius, acc_radius):
        """
        Write calibration data (define as 16 bit signed hex).

        :param acc_offset:
        :param mag_offset:
        :param gyr_offset:
        :param mag_radius:
        :param acc_radius:
        """

        return

        # Must switch to config mode to write out
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_CONFIG]))):
            self.node.get_logger().error('Unable to set IMU into config mode')
        sleep(0.025)
        try:
            self.con.transmit(registers.ACCEL_OFFSET_X_LSB_ADDR, 1, bytes([acc_offset.value[0] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_X_MSB_ADDR, 1, bytes([(acc_offset.value[0] >> 8) & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Y_LSB_ADDR, 1, bytes([acc_offset.value[1] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Y_MSB_ADDR, 1, bytes([(acc_offset.value[1] >> 8) & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Z_LSB_ADDR, 1, bytes([acc_offset.value[2] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Z_MSB_ADDR, 1, bytes([(acc_offset.value[2] >> 8) & 0xFF]))

            self.con.transmit(registers.ACCEL_RADIUS_LSB_ADDR, 1, bytes([acc_radius.value & 0xFF]))
            self.con.transmit(registers.ACCEL_RADIUS_MSB_ADDR, 1, bytes([(acc_radius.value >> 8) & 0xFF]))

            self.con.transmit(registers.MAG_OFFSET_X_LSB_ADDR, 1, bytes([mag_offset.value[0] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_X_MSB_ADDR, 1, bytes([(mag_offset.value[0] >> 8) & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Y_LSB_ADDR, 1, bytes([mag_offset.value[1] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Y_MSB_ADDR, 1, bytes([(mag_offset.value[1] >> 8) & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Z_LSB_ADDR, 1, bytes([mag_offset.value[2] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Z_MSB_ADDR, 1, bytes([(mag_offset.value[2] >> 8) & 0xFF]))

            self.con.transmit(registers.MAG_RADIUS_LSB_ADDR, 1, bytes([mag_radius.value & 0xFF]))
            self.con.transmit(registers.MAG_RADIUS_MSB_ADDR, 1, bytes([(mag_radius.value >> 8) & 0xFF]))

            self.con.transmit(registers.GYRO_OFFSET_X_LSB_ADDR, 1, bytes([gyr_offset.value[0] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_X_MSB_ADDR, 1, bytes([(gyr_offset.value[0] >> 8) & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Y_LSB_ADDR, 1, bytes([gyr_offset.value[1] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Y_MSB_ADDR, 1, bytes([(gyr_offset.value[1] >> 8) & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Z_LSB_ADDR, 1, bytes([gyr_offset.value[2] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Z_MSB_ADDR, 1, bytes([(gyr_offset.value[2] >> 8) & 0xFF]))

            return True
        except Exception:  # noqa: B902
            return False

    def calibration_request_callback(self, request, response):
        return
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_CONFIG]))):
            self.node.get_logger().warn('Unable to set IMU into config mode.')
        sleep(0.025)
        calib_data = self.get_calib_data()
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_NDOF]))):
            self.node.get_logger().warn('Unable to set IMU operation mode into operation mode.')
        response.success = True
        response.message = str(calib_data)
        return response