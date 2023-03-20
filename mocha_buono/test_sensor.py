import time
import board
import adafruit_bno055

sensor = adafruit_bno055.BNO055_I2C(board.I2C())

def unpackBytesToFloat(start, end):
    return float(struct.unpack('h', struct.pack('BB', start, end))[0])

while True:
    print("Accelerometer (m/s^2): {}".format(sensor.acceleration)) # (-0.09, 6.67, 6.72)
    print("Magnetometer (microteslas): {}".format(sensor.magnetic)) # (25.375, -49.1875, -20.5)
    print("Gyroscope (rad/sec): {}".format(sensor.gyro)) # (0.001090830782496456, -0.001090830782496456, -0.001090830782496456)
    print("Euler angle: {}".format(sensor.euler)) # (0.0, -0.5, -44.5625)
    print("Quaternion: {}".format(sensor.quaternion)) # (0.9251708984375, 0.37945556640625, 0.0050048828125, 0.0)
    print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration)) # (0.0, -0.24, -0.22)
    print("Gravity (m/s^2): {}".format(sensor.gravity)) # (-0.09, 6.88, 6.98)
    print()
    time.sleep(0.1)
