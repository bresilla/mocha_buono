from rclpy.node import Node

#: Default calibration values (taken from desk test approximation.) [x y z]
#: Signed hex 16 bit representation

#: +/- 2000 units (at max 2G)    (1 unit = 1 mg = 1 LSB = 0.01 m/s2)
DEFAULT_OFFSET_ACC = [0xFFEC, 0x00A5, 0xFFE8]
#: +/- 6400 units                (1 unit = 1/16 uT)
DEFAULT_OFFSET_MAG = [0xFFB4, 0xFE9E, 0x027D]
#: +/- 2000 units up to 32000 (dps range dependent)               (1 unit = 1/16 dps)
DEFAULT_OFFSET_GYR = [0x0002, 0xFFFF, 0xFFFF]

DEFAULT_RADIUS_MAG = 0x0
DEFAULT_RADIUS_ACC = 0x3E8
#: Sensor standard deviation squared (^2) defaults [x, y, z]
#: Used to get covariance matrices (stddev^2 = variance)
#: values taken from this ROS1 driver from octanis:
#: https://github.com/Octanis1/bosch_imu_driver/commit/d1132e27ecff46a63c128f7ecacc245c98b2811a
DEFAULT_VARIANCE_ACC = [0.017, 0.017, 0.017]
DEFAULT_VARIANCE_ANGULAR_VEL = [0.04, 0.04, 0.04]
DEFAULT_VARIANCE_ORIENTATION = [0.0159, 0.0159, 0.0159]
DEFAULT_VARIANCE_MAG = [0.0, 0.0, 0.0]

class SensorParameters:
    """
    ROS2 Node Parameter Handling.

    https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters
    https://index.ros.org/doc/ros2/Tutorials/Using-Parameters-In-A-Class-Python/

    Start the node with parameters from yml file:
    ros2 run mocha_buono bno055

    with the following arguments:
    --ros-args --params-file <workspace>/src/mocha_buono/mocha_buono/params/mocha_buono.yaml
    """
    def __init__(self, node: Node):
        node.get_logger().info('Initializing parameters')
        # Declare parameters of the ROS2 node and their default values:

        # The topic prefix to use (can be empty if not required)
        node.declare_parameter(name='ros_topic_prefix', value='mocha_buono/')

        # tf frame id
        node.declare_parameter('frame_id', value='mocha_buono')
        # sensor operation mode
        node.declare_parameter('operation_mode', value=0x0C)
        # placement_axis_remap defines the position and orientation of the sensor mount
        node.declare_parameter('placement_axis_remap', value='P1')
        # scaling factor for acceleration
        node.declare_parameter('acc_factor', value=100.0)
        # scaling factor for magnetometer
        node.declare_parameter('mag_factor', value=16000000.0)
        # scaling factor for gyroscope
        node.declare_parameter('gyr_factor', value=900.0)

        # Node timer frequency in Hz, defining how often sensor data is requested
        node.declare_parameter('data_query_frequency', value=30)
        # Node timer frequency in Hz, defining how often calibration status data is requested
        node.declare_parameter('calib_status_frequency', value=0.1)

        node.declare_parameter('variance_acc', value=DEFAULT_VARIANCE_ACC)
        node.declare_parameter('variance_angular_vel', value=DEFAULT_VARIANCE_ANGULAR_VEL)
        node.declare_parameter('variance_orientation', value=DEFAULT_VARIANCE_ORIENTATION)
        node.declare_parameter('variance_mag', value=DEFAULT_VARIANCE_MAG)

        # get the parameters - requires CLI arguments '--ros-args --params-file <parameter file>'
        node.get_logger().info('Parameters set to:')

        try:
            self.ros_topic_prefix = node.get_parameter('ros_topic_prefix')
            node.get_logger().info('\tros_topic_prefix:\t"%s"' % self.ros_topic_prefix.value)

            self.frame_id = node.get_parameter('frame_id')
            node.get_logger().info('\tframe_id:\t\t"%s"' % self.frame_id.value)
        
            self.operation_mode = node.get_parameter('operation_mode')
            node.get_logger().info('\toperation_mode:\t\t"%s"' % self.operation_mode.value)

            self.placement_axis_remap = node.get_parameter('placement_axis_remap')
            node.get_logger().info('\tplacement_axis_remap:\t"%s"'
                                   % self.placement_axis_remap.value)

            self.acc_factor = node.get_parameter('acc_factor')
            node.get_logger().info('\tacc_factor:\t\t"%s"' % self.acc_factor.value) 

            self.mag_factor = node.get_parameter('mag_factor')
            node.get_logger().info('\tmag_factor:\t\t"%s"' % self.mag_factor.value)

            self.gyr_factor = node.get_parameter('gyr_factor')
            node.get_logger().info('\tgyr_factor:\t\t"%s"' % self.gyr_factor.value)

            self.data_query_frequency = node.get_parameter('data_query_frequency')
            node.get_logger().info('\tdata_query_frequency:\t"%s"' % self.data_query_frequency.value)

            self.calib_status_frequency = node.get_parameter('calib_status_frequency')
            node.get_logger().info('\tcalib_status_frequency:\t"%s"' % self.calib_status_frequency.value)

            self.variance_acc = node.get_parameter('variance_acc')
            node.get_logger().info('\tvariance_acc:\t\t"%s"' % self.variance_acc.value)
            self.variance_angular_vel = node.get_parameter('variance_angular_vel')
            node.get_logger().info('\tvariance_angular_vel:\t"%s"' % self.variance_angular_vel.value)
            self.variance_orientation = node.get_parameter('variance_orientation')
            node.get_logger().info('\tvariance_orientation:\t"%s"' % self.variance_orientation.value)
            self.variance_mag = node.get_parameter('variance_mag')
            node.get_logger().info('\tvariance_mag:\t\t"%s"' % self.variance_mag.value)

        except Exception as e:
            node.get_logger().warn('Could not get parameters...setting variables to default')
            node.get_logger().warn('Error: "%s"' % e)