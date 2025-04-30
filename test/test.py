import smbus
import math
import time
import RPi.GPIO as GPIO
import pygame

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

# 定义控制震动马达的 GPIO 引脚
MOTOR_PIN = 18

# 设置 GPIO 模式为 BCM
GPIO.setmode(GPIO.BCM)
# 设置震动马达的 GPIO 引脚为 PWM 模式
GPIO.setup(MOTOR_PIN, GPIO.OUT)
pwm = GPIO.PWM(MOTOR_PIN, 100)  # 初始化 PWM，频率为 100Hz
# 启动 PWM，初始占空比为 0
pwm.start(0)

# 初始化pygame的mixer模块
pygame.mixer.init()

# 加载音频文件（路径根据实际情况修改）
pygame.mixer.music.load("/home/pi/test/two.mp3")

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

def detect_fall(Ax, Ay, Az, Gx, Gy, Gz):
    # 计算加速度的模
    accel_magnitude = math.sqrt(Ax**2 + Ay**2 + Az**2)
    # 计算陀螺仪的模
    gyro_magnitude = math.sqrt(Gx**2 + Gy**2 + Gz**2)

    # 简单的摔倒检测规则：加速度模小于0.5g且陀螺仪模大于100°/s
    if accel_magnitude > 2 and gyro_magnitude > 150:
        return True
    return False

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

        # 检测是否摔倒
        if detect_fall(Ax, Ay, Az, Gx, Gy, Gz):
            print("检测到摔倒！")
            # 播放音频
            pygame.mixer.music.play()            
            # 控制震动马达以最大强度震动
            pwm.ChangeDutyCycle(100)
            time.sleep(3)  # 震动5秒
            pwm.ChangeDutyCycle(0)  # 停止震动
        else:
            # 未检测到摔倒，震动马达停止
            pwm.ChangeDutyCycle(0)

        time.sleep(0.1)  # 等待0.1秒再进行下一次测量

except KeyboardInterrupt:
    print("程序终止")
finally:
    # 停止 PWM
    pwm.stop()
    # 清理 GPIO 设置
    GPIO.cleanup()