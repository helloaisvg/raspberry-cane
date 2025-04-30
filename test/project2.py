import RPi.GPIO as GPIO
import time
import pyttsx3

# 设置GPIO引脚
ENA = 13  # L298N的PWM控制引脚
IN1 = 19  # L298N的方向引脚1
IN2 = 26  # L298N的方向引脚2

# 超声波传感器的引脚
TRIG1 = 23  # 超声波传感器1的触发引脚
ECHO1 = 24  # 超声波传感器1的回波引脚
TRIG2 = 17  # 超声波传感器2的触发引脚
ECHO2 = 27  # 超声波传感器2的回波引脚

# 初始化GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# 设置L298N电机控制引脚
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# 设置超声波传感器引脚
GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)
GPIO.setup(TRIG2, GPIO.OUT)
GPIO.setup(ECHO2, GPIO.IN)

# 设置PWM频率并初始化PWM信号
freq = 500
speed = 0
pwm = GPIO.PWM(ENA, freq)
pwm.start(speed)

# 初始化语音引擎
engin = pyttsx3.init("espeak")
engin.setProperty('voice', "en")  # 设置语音为英文
engin.setProperty('volume', 0.4)  # 设置音量
engin.setProperty('rate', 175)    # 设置语速

# 超声波测距函数
def distance(trigger_pin, echo_pin):
    # 发送触发信号，开始测距
    GPIO.output(trigger_pin, False)
    time.sleep(0.2)
    
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
    
    while GPIO.input(echo_pin) == 1:
        end_time = time.time()

    time_elapsed = end_time - start_time
    distance = time_elapsed * 17150  # 计算距离，单位为厘米
    return distance

# 电机正转
def motor_forward():
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    engin.say("Motor is moving forward")
    engin.runAndWait()

# 电机反转
def motor_backward():
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    engin.say("Motor is moving backward")
    engin.runAndWait()

# 电机停止
def motor_stop():
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    engin.say("Motor has stopped")
    engin.runAndWait()

# 主循环
try:
    while True:
        # 获取两个超声波传感器的距离
        dist1 = distance(TRIG1, ECHO1)
        dist2 = distance(TRIG2, ECHO2)
        
        print("Sensor 1 Distance:", dist1, "cm")
        print("Sensor 2 Distance:", dist2, "cm")
        
        # 判断两个传感器的距离，根据距离值决定电机行为
        if dist1 < 20 and dist2 < 20:  # 如果两个传感器的距离都小于20cm
            motor_stop()
        elif dist1 < 20:  # 如果第一个传感器距离小于20cm
            motor_backward()
        elif dist2 < 20:  # 如果第二个传感器距离小于20cm
            motor_forward()
        else:
            motor_stop()  # 没有物体，电机停止

        time.sleep(1)  # 每秒钟测量一次

except KeyboardInterrupt:
    print("Program interrupted")
finally:
    pwm.stop()  # 停止PWM
    GPIO.cleanup()  # 清理GPIO资源
