import smbus
import math
import time
import RPi.GPIO as GPIO
import pygame

import serial
import pynmea2

ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=9600,
    timeout=1
)

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
# 定义超声波传感器的 GPIO 引脚
TRIG = 23
ECHO = 24


# 设置 GPIO 模式为 BCM
GPIO.setmode(GPIO.BCM)

# 设置超声波传感器的 GPIO 引脚
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# 设置震动马达的 GPIO 引脚为 PWM 模式
GPIO.setup(MOTOR_PIN, GPIO.OUT)
pwm = GPIO.PWM(MOTOR_PIN, 100)  # 初始化 PWM，频率为 100Hz
# 启动 PWM，初始占空比为 0
pwm.start(0)

# 初始化pygame的mixer模块
pygame.mixer.init()

# 加载摔倒检测音频文件（路径根据实际情况修改）
fall_audio_path = "/home/pi/code/two.mp3"
pygame.mixer.music.load(fall_audio_path)
# 加载障碍物检测音频文件
obstacle_audio_path = "/home/pi/code/one.mp3"
pygame.mixer.music.load(obstacle_audio_path)

# 用于移动平均滤波的参数
FILTER_WINDOW_SIZE = 5  # 滤波窗口大小
distance_buffer = []    # 存储最近的距离测量值

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

    # 简单的摔倒检测规则：加速度模大于2且陀螺仪模大于150°/s
    if accel_magnitude > 2 and gyro_magnitude > 150:
        return True
    return False

def measure_distance():
    # 发送超声波脉冲
    GPIO.output(TRIG, False)
    time.sleep(0.1)  # 等待0.1秒，确保传感器稳定
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # 接收超声波回波
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # 计算距离
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 34300 / 2  # 声音速度为34300厘米/秒
    return distance

MPU_Init()
print("MPU6050 初始化完成")

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            if line.startswith('$'):
                try:
                    msg = pynmea2.parse(line)
                    if isinstance(msg, pynmea2.GGA):
                        print(f"纬度: {msg.latitude}, 经度: {msg.longitude}")
                except pynmea2.ParseError as e:
                    print(f"解析错误定位失败: {e}")
                time.sleep(0.1) 
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

        #print("加速度计: Ax=%.2f g, Ay=%.2f g, Az=%.2f g" % (Ax, Ay, Az))
        #print("陀螺仪: Gx=%.2f °/s, Gy=%.2f °/s, Gz=%.2f °/s" % (Gx, Gy, Gz))

        # 检测是否摔倒
        if detect_fall(Ax, Ay, Az, Gx, Gy, Gz):
            print("检测到摔倒！")
            # 加载摔倒检测音频并播放
            pygame.mixer.music.load(fall_audio_path)
            pygame.mixer.music.play()
            # 控制震动马达以最大强度震动
            pwm.ChangeDutyCycle(100)
            time.sleep(3)  # 震动3秒
            pwm.ChangeDutyCycle(0)  # 停止震动
        else:
            # 未检测到摔倒，进行障碍物检测
            distance = measure_distance()
            # 将新的距离测量值添加到缓冲区
            distance_buffer.append(distance)
            if len(distance_buffer) > FILTER_WINDOW_SIZE:
                distance_buffer.pop(0)  # 移除最旧的测量值

            # 计算移动平均值
            average_distance = sum(distance_buffer) / len(distance_buffer)

            # 打印滤波后的距离
            print("滤波后距离: %.2f 厘米" % average_distance)

            # 根据滤波后的距离调整震动马达的占空比
            if average_distance > 60:
                duty_cycle = 0
            elif average_distance > 10:
                # 修正占空比计算方式
                duty_cycle = int((60 - average_distance) / (60 - 10) * 100)
                # 加载障碍物检测音频并播放
                pygame.mixer.music.load(obstacle_audio_path)
                pygame.mixer.music.play()
                print("前方有障碍！")
                time.sleep(0.5)
            else:
                duty_cycle = 100
                pygame.mixer.music.play()
                print("前方有障碍！")
                time.sleep(0.5)

            # 改变 PWM 的占空比
            pwm.ChangeDutyCycle(duty_cycle)

        time.sleep(0.1)  # 等待0.1秒再进行下一次测量

except KeyboardInterrupt:
    print("程序终止")
finally:
    # 停止 PWM
    pwm.stop()
    # 清理 GPIO 设置
    GPIO.cleanup()