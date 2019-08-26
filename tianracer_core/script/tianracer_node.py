#!/usr/bin/env python
import math
import sys

import rospy
from SerialDataGateway import SerialDataGateway
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String, Header


class TianbotRacecar(object):
    """Class to handle serial data from TianbotRacecar and converted to ROS topics"""

    def __init__(self):
        rospy.loginfo("Initializing TianbotRacecar Class")

        # Sensor variables
        self._counter = 0
        self._battery_value = 0

        self._qx = 0
        self._qy = 0
        self._qz = 0
        self._qw = 0

        self._servo = 90
        self._motor = 1500
        self._speed_message = 's %d %d\r' % (int(self._servo), int(self._motor))
        self._robot_heading = 0

        # Get serial port and baud rate of Tiva C TianBot_RACECAR
        port = rospy.get_param("~port", "/dev/ttyUSB1")
        baudRate = int(rospy.get_param("~baudrate", 115200))
        self._servo_direction = int(rospy.get_param("~servo_direction", 1))
        self._motor_direction = int(rospy.get_param("~motor_direction", 1))

        rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))
        # Initializing SerialDataGateway with port, baudrate and callback function to handle serial data
        self._SerialDataGateway = SerialDataGateway(port, baudRate, self.handle_received_line)
        rospy.loginfo("Started serial communication")

        # Publisher for Battery level(for upgrade purpose)
        self._battery_pub = rospy.Publisher('battery_level', Float32, queue_size=10)

        # Publisher for entire serial data
        self._SerialPublisher = rospy.Publisher('serial', String, queue_size=10)

        # Subscribers and Publishers of IMU data topic
        self._frame_id = 'IMU_link'
        self._cal_offset = 0.0
        self._orientation = 0.0
        self._cal_buffer = []
        self._cal_buffer_length = 1000
        self._imu_data = Imu(header=rospy.Header(frame_id="IMU_link"))
        self._imu_data.orientation_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        self._imu_data.angular_velocity_covariance = [0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02]
        self._imu_data.linear_acceleration_covariance = [0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04]
        self._gyro_measurement_range = 300.0
        self._gyro_scale_correction = 1.35
        self._imu_pub = rospy.Publisher('imu_data', Imu, queue_size=10)
        self._deltat = 0
        self._last_update = 0

        # New addon for computing quaternion
        self._pi = 3.1415926535
        self._gyro_meas_error = float(self._pi * (40 / 180))
        self._beta = float(math.sqrt(3 / 4) * self._gyro_meas_error)
        self._gyro_meas_drift = float(self._pi * (2 / 180))
        self._zeta = float(math.sqrt(3 / 4) * self._gyro_meas_drift)
        self._beta = math.sqrt(3 / 4) * self._gyro_meas_error
        self._q = [1, 0, 0, 0]

        # Speed subscriber
        self._speed_sub = rospy.Subscriber('/car/cmd_vel', Twist, self.update_speed)

        # Timer for serial communication
        self._timer = rospy.Timer(rospy.Duration(0.05), self.car_controller)

    def update_speed(self, speed):
        self._motor = 1500 + self._motor_direction * (speed.linear.x - 1500)
        self._servo = 90 + self._servo_direction * (speed.angular.z - 90)
        self._speed_message = 's %d %d\r' % (int(self._servo), int(self._motor))

    def car_controller(self, tdat):
        self.write_serial(self._speed_message)

    def handle_received_line(self, line):
        """Calculate orientation from accelerometer and gyrometer"""
        self._counter = self._counter + 1
        self._SerialPublisher.publish(String(str(self._counter) + ", in:  " + line))

        if line:
            lineParts = line.split('\t')
            try:
                if lineParts[0] == 'b':
                    self._battery_value = float(lineParts[1])
                    self._battery_pub.publish(self._battery_value)

                if lineParts[0] == 'i':
#                    self._qx, self._qy, self._qz, self._qw, \
#                    self._gx, self._gy, self._gz, \
#                    self._ax, self._ay, self.az = [float(i) for i in lineParts[1:11]]
                    self._qx = float(lineParts[1])
                    self._qy = float(lineParts[2])
                    self._qz = float(lineParts[3])
                    self._qw = float(lineParts[4])
                    self._gx = float(lineParts[5])
                    self._gy = float(lineParts[6])
                    self._gz = float(lineParts[7])
                    self._ax = float(lineParts[8])
                    self._ay = float(lineParts[9])
                    self._az = float(lineParts[10])

                    imu_msg = Imu()
                    header = Header()
                    header.stamp = rospy.Time.now()
                    header.frame_id = self._frame_id
                    imu_msg.header = header

                    imu_msg.orientation_covariance = self._imu_data.orientation_covariance
                    imu_msg.angular_velocity_covariance = self._imu_data.angular_velocity_covariance
                    imu_msg.linear_acceleration_covariance = self._imu_data.linear_acceleration_covariance

                    imu_msg.orientation.x = self._qx
                    imu_msg.orientation.y = self._qy
                    imu_msg.orientation.z = self._qz
                    imu_msg.orientation.w = self._qw
                    imu_msg.angular_velocity.x = self._gx
                    imu_msg.angular_velocity.y = self._gy
                    imu_msg.angular_velocity.z = self._gz
                    imu_msg.linear_acceleration.x = self._ax
                    imu_msg.linear_acceleration.y = self._ay
                    imu_msg.linear_acceleration.z = self._az
                    # q_rot = quaternion_from_euler(self.pi, -self.pi/2, 0)
                    # q_ori = Quaternion(self._qx, self._qy, self._qz, self._qw)
                    # imu_msg.orientation = quaternion_multiply(q_ori, q_rot)
                    self._imu_pub.publish(imu_msg)
            except:
                rospy.logwarn("Error in Sensor values")
                rospy.logwarn(lineParts)

    def write_serial(self, message):
        self._SerialPublisher.publish(String(str(self._counter) + ", out: " + message))
        self._SerialDataGateway.Write(message)

    def start(self):
        rospy.logdebug("Starting")
        self._SerialDataGateway.Start()

    def stop(self):
        rospy.logdebug("Stopping")
        self.reset()
        self._SerialDataGateway.Stop()

    def reset(self):
        self._SerialDataGateway.Reset()
        rospy.sleep(3)


if __name__ == '__main__':
    rospy.init_node('tianracer_core', anonymous=True)
    tianbot_racecar=TianbotRacecar()

    try:
        while True:
            try:
                tianbot_racecar.start()
            except:
                rospy.logwarn('Device not connected yet, will try again...')
                rospy.sleep(3)
            else:
                break
    except KeyboardInterrupt:
        rospy.logwarn('Bye!')
        sys.exit(0)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("Error in main function")
    finally:
        rospy.loginfo('Do some cleaning before exit')
        tianbot_racecar.stop()
