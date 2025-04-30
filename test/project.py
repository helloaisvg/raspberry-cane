import RPi.GPIO as GPIO
import time
import pygame


# 初始化pygame的mixer模块
pygame.mixer.init()

# 加载音频文件（路径根据实际情况修改）
pygame.mixer.music.load("/home/pi/test/one.mp3")


# 设置 GPIO 模式为 BCM
GPIO.setmode(GPIO.BCM)

# 定义控制震动马达的 GPIO 引脚
MOTOR_PIN = 18
# 定义超声波传感器的 GPIO 引脚
TRIG = 23
ECHO = 24

# 设置震动马达的 GPIO 引脚为 PWM 模式
GPIO.setup(MOTOR_PIN, GPIO.OUT)
pwm = GPIO.PWM(MOTOR_PIN, 100)  # 初始化 PWM，频率为 100Hz

# 设置超声波传感器的 GPIO 引脚
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# 用于移动平均滤波的参数
FILTER_WINDOW_SIZE = 5  # 滤波窗口大小
distance_buffer = []    # 存储最近的距离测量值

try:
    # 启动 PWM，初始占空比为 0
    pwm.start(0)

    while True:
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
            # 播放音频
            pygame.mixer.music.play()
            print("前方有障碍！")
        else:
            duty_cycle = 100

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