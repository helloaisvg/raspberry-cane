import time
import RPi.GPIO as GPIO
import pygame
import threading

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

IN1_PIN1 = 19
IN2_PIN1 = 16

# 设置GPIO引脚
TRIG1 = 23
ECHO1 = 24
GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)

GPIO.setup(IN1_PIN1, GPIO.OUT)
p1 = GPIO.PWM(IN1_PIN1, 50)
p1.start(0)

GPIO.setup(IN2_PIN1, GPIO.OUT)
p2 = GPIO.PWM(IN2_PIN1, 50)
p2.start(0)

# 播放音频的函数
def play_audio():
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():
        time.sleep(0.1)

speed = 40

# 电机控制函数
def forward(time_sleep):
    p1.ChangeDutyCycle(speed)
    p2.ChangeDutyCycle(0)
    time.sleep(time_sleep)
    
def reverse(time_sleep):
    p1.ChangeDutyCycle(0)
    p2.ChangeDutyCycle(speed)
    time.sleep(time_sleep)

def stop():
    p1.ChangeDutyCycle(0)
    p2.ChangeDutyCycle(0)
    time.sleep(0.001)

# 超声波测距函数
def distance(trigger_pin, echo_pin):
    GPIO.output(trigger_pin, False)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    start_time = time.time()
    while GPIO.input(echo_pin) == 0:
        start_time = time.time()

    while GPIO.input(echo_pin) == 1:
        end_time = time.time()

    time_elapsed = end_time - start_time
    distance = time_elapsed * 17150  # 计算距离，单位为厘米

    if distance < 2 or distance > 400:
        return -1  # 如果超出有效范围，返回-1

    return distance

# 初始化pygame的mixer模块
pygame.mixer.init()
pygame.mixer.music.load("/home/pi/project/one.mp3")

# 定义全局变量和锁
current_distance = -1
distance_lock = threading.Lock()

# 超声波测距线程
def distance_thread():
    global current_distance
    while True:
        dist1 = distance(TRIG1, ECHO1)
        with distance_lock:
            current_distance = dist1
        time.sleep(0.5)  # 每隔0.5秒测量一次

# 主线程控制电机和播放音频
def main():
    global current_distance

    try:
        while True:
            with distance_lock:
                dist1 = current_distance
            print("Sensor 1 Distance:", dist1, "cm")

            if dist1 != -1:
                forward(0.6)
                stop()
                time.sleep(0.5)

                with distance_lock:
                    dist1 = current_distance
                print("Sensor 1 Distance:", dist1, "cm")

                reverse(0.6)
                stop()
                time.sleep(0.5)

                if dist1 <= 20 and dist1 > 0:
                    # 播放语音警告
                    time.sleep(0.2)
                    play_audio()
                    print("前方有障碍")
                    time.sleep(1.5)  # 等待1秒再进行下一次测量
            else:
                print("超声波测量无效，跳过本次循环")
                time.sleep(0.5)

    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__ == "__main__":
    # 启动超声波测距线程
    distance_threading = threading.Thread(target=distance_thread, daemon=True)
    distance_threading.start()

    # 启动主程序
    main()
