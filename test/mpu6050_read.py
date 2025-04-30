import smbus
import math
import time

# MPU6050寄存器地址
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# I2C总线初始化
bus = smbus.SMBus(1)
Device_Address = 0x68

def MPU_Init():
    # 写入采样率寄存器
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    # 写入电源管理寄存器
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    # 写入配置寄存器
    bus.write_byte_data(Device_Address, CONFIG, 0)
    # 写入陀螺仪配置寄存器
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 0)
    # 写入加速度计配置寄存器
    bus.write_byte_data(Device_Address, ACCEL_CONFIG, 0)
    # 写入中断使能寄存器
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    # 读取高字节和低字节数据
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)
    # 合并高低字节数据
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value

MPU_Init()
print("MPU6050 初始化完成")

try:
    while True:
        # 读取加速度计数据
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_XOUT_H + 2)
        acc_z = read_raw_data(ACCEL_XOUT_H + 4)
        # 读取陀螺仪数据
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_XOUT_H + 2)
        gyro_z = read_raw_data(GYRO_XOUT_H + 4)
        # 计算加速度计的重力加速度值
        Ax = acc_x / 16384.0
        Ay = acc_y / 16384.0
        Az = acc_z / 16384.0
        # 计算陀螺仪的角速度值
        Gx = gyro_x / 131.0
        Gy = gyro_y / 131.0
        Gz = gyro_z / 131.0

        print("加速度计: Ax=%.2f g, Ay=%.2f g, Az=%.2f g" % (Ax, Ay, Az))
        print("陀螺仪: Gx=%.2f °/s, Gy=%.2f °/s, Gz=%.2f °/s" % (Gx, Gy, Gz))
        print("-" * 30)
        time.sleep(0.5)

except KeyboardInterrupt:
    print("程序终止")
	
	
	
	
	
	
	
'''
from mpu6050 import mpu6050
import time
 
sensor = mpu6050(0x68)
 
while True:
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    temp = sensor.get_temp()
 
    print("Accelerometer data")
    print("x: " + str(accel_data['x']))
    print("y: " + str(accel_data['y']))
    print("z: " + str(accel_data['z']))
 
    print("Gyroscope data")
    print("x: " + str(gyro_data['x']))
    print("y: " + str(gyro_data['y']))
    print("z: " + str(gyro_data['z']))
 
    print("Temp: " + str(temp) + " C")
    time.sleep(0.5)
'''