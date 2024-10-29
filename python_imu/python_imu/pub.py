#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

buf_length = 11
RxBuff = [0] * buf_length

ACCData = [0.0] * 8
GYROData = [0.0] * 8
AngleData = [0.0] * 8
FrameState = 0
CheckSum = 0

start = 0
data_length = 0

acc = [0.0] * 3
gyro = [0.0] * 3
Angle = [0.0] * 3

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=0.2)
        self.get_logger().info(f"Serial port opened: {self.serial_port.is_open}")
        self.timer = self.create_timer(0.001, self.read_imu_data)  # Adjust rate as needed

    def read_imu_data(self):
        if self.serial_port.in_waiting > 0:
            RXdata = self.serial_port.read(1)
            RXdata = int(RXdata.hex(), 16)
            self.due_data(RXdata)

    def due_data(self, inputdata):
        global start, CheckSum, data_length

        if inputdata == 0x55 and start == 0:
            start = 1
            data_length = 11
            CheckSum = 0
            for i in range(11):
                RxBuff[i] = 0

        if start == 1:
            CheckSum += inputdata
            RxBuff[buf_length - data_length] = inputdata
            data_length -= 1

            if data_length == 0:
                CheckSum = (CheckSum - inputdata) & 0xff
                start = 0
                self.get_data_deal(RxBuff)

    def get_data_deal(self, list_buf):
        global acc, gyro, Angle

        if list_buf[buf_length - 1] != CheckSum:
            return

        if list_buf[1] == 0x51:  # Acceleration
            for i in range(6):
                ACCData[i] = list_buf[2 + i]
            acc = self.get_acc(ACCData)

        elif list_buf[1] == 0x52:  # Angular velocity
            for i in range(6):
                GYROData[i] = list_buf[2 + i]
            gyro = self.get_gyro(GYROData)

        elif list_buf[1] == 0x53:  # Angle
            for i in range(6):
                AngleData[i] = list_buf[2 + i]
            Angle = self.get_angle(AngleData)

        self.publish_imu_message()

    def get_acc(self, datahex):
        axl, axh, ayl, ayh, azl, azh = datahex[:6]
        k_acc = 16.0
        acc_x = (axh << 8 | axl) / 32768.0 * k_acc
        acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
        acc_z = (azh << 8 | azl) / 32768.0 * k_acc
        return self.adjust_acc(acc_x, k_acc), self.adjust_acc(acc_y, k_acc), self.adjust_acc(acc_z, k_acc)

    def adjust_acc(self, acc, k_acc):
        return acc - 2 * k_acc if acc >= k_acc else acc

    def get_gyro(self, datahex):
        wxl, wxh, wyl, wyh, wzl, wzh = datahex[:6]
        k_gyro = 2000.0
        gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
        gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
        gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
        return self.adjust_gyro(gyro_x, k_gyro), self.adjust_gyro(gyro_y, k_gyro), self.adjust_gyro(gyro_z, k_gyro)

    def adjust_gyro(self, gyro, k_gyro):
        return gyro - 2 * k_gyro if gyro >= k_gyro else gyro

    def get_angle(self, datahex):
        rxl, rxh, ryl, ryh, rzl, rzh = datahex[:6]
        k_angle = 180.0
        angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
        angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
        angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
        return self.adjust_angle(angle_x, k_angle), self.adjust_angle(angle_y, k_angle), self.adjust_angle(angle_z, k_angle)

    def adjust_angle(self, angle, k_angle):
        return angle - 2 * k_angle if angle >= k_angle else angle

    def publish_imu_message(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"  # Set the frame ID

        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = acc
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = gyro
        imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z = Angle
        self.publisher_.publish(imu_msg)
        self.get_logger().info("IMU Data Published")

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
