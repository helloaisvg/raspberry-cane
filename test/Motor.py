import RPi.GPIO as GPIO         # 引入GPIO模块
import time                     # 引入time模块

ENA = 13
IN1 = 19
IN2 = 26

if __name__ == '__main__':
    try:
        # 初始化
        GPIO.setmode(GPIO.BCM)              # 使用BCM编号方式
        GPIO.setup(ENA, GPIO.OUT)           # 将ENA对应的GPIO引脚设置为输出模式
        GPIO.setup(IN1, GPIO.OUT)           # 将IN1对应的GPIO引脚设置为输出模式
        GPIO.setup(IN2, GPIO.OUT)           # 将IN2对应的GPIO引脚设置为输出模式

        freq = 500
        speed = 0
        pwm = GPIO.PWM(ENA, freq)           # 设置向ENA输入PWM脉冲信号，频率为freq并创建PWM对象
        pwm.start(speed)                    # 以speed的初始占空比开始向ENA输入PWM脉冲信号

        while True:
            # 将电机设置为正向转动
            GPIO.output(IN1, False)         # 将IN1设置为0
            GPIO.output(IN2, True)          # 将IN2设置为1

            # 通过改变PWM占空比，让电机转速不断加快
            for speed in range(0, 100, 5):
                pwm.ChangeDutyCycle(speed)  # 改变PWM占空比
                time.sleep(1)

            # 将电机设置为反向转动
            GPIO.output(IN1, True)          # 将IN1设置为1
            GPIO.output(IN2, False)         # 将IN2设置为0

            # 通过改变PWM占空比，让电机转速不断加快
            for speed in range(0, 100, 5):
                pwm.ChangeDutyCycle(speed)  # 改变PWM占空比
                time.sleep(1)
    finally:
        pwm.stop()                          # 停止PWM
        GPIO.cleanup()                      # 清理释放GPIO资源，将GPIO复位

