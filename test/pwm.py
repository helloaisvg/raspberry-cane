import RPi.GPIO as GPIO
import time

# 设置 GPIO 模式为 BCM
GPIO.setmode(GPIO.BCM)

# 定义控制震动马达的 GPIO 引脚
MOTOR_PIN = 18

# 设置 GPIO 引脚为 PWM 模式
GPIO.setup(MOTOR_PIN, GPIO.OUT)
pwm = GPIO.PWM(MOTOR_PIN, 100)  # 初始化 PWM，频率为 100Hz

try:
    # 启动 PWM，初始占空比为 0
    pwm.start(0)

    while True:
        # 逐渐增加震动强度
        for duty_cycle in range(0, 101, 5):
            pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.1)

        # 逐渐降低震动强度
        for duty_cycle in range(100, -1, -5):
            pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.1)

except KeyboardInterrupt:
    print("程序终止")
finally:
    # 停止 PWM
    pwm.stop()
    # 清理 GPIO 设置
    GPIO.cleanup()