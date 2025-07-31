#!/usr/bin/env python3
import rospy
import serial

def send_command():
    rospy.init_node('serial_screen_node', anonymous=True)
    port = "/dev/ttyUSB0"  # 替换为你的串口设备路径
    baudrate = 115200  # 设置波特率

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        rospy.loginfo("Serial Port initialized")
    except serial.SerialException as e:
        rospy.logerr(f"Unable to open port: {e}")
        return

    # 构造原始字节序列
    command = b"n0.val=10\xff\xff\xff"

    # 发送原始字节序列
    ser.write(command)

    # 关闭串口
    ser.close()

if __name__ == '__main__':
    try:
        send_command()
    except rospy.ROSInterruptException:
        pass
