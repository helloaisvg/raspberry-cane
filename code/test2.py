import smbus
import math
import time
import RPi.GPIO as GPIO
import pygame
import serial
import pynmea2
import cv2
import numpy as np
import onnxruntime as ort

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

# 加载音频文件
fall_audio_path = "/home/pi/code/two.mp3"
obstacle_audio_path = "/home/pi/code/one.mp3"
person_audio_path = "/home/pi/code/person.mp3"
car_audio_path = "/home/pi/code/car.mp3"
light_audio_path = "/home/pi/code/light.mp3"

# 加载所有音频文件
pygame.mixer.music.load(fall_audio_path)
pygame.mixer.music.load(obstacle_audio_path)
pygame.mixer.music.load(person_audio_path)
pygame.mixer.music.load(car_audio_path)
pygame.mixer.music.load(light_audio_path)

# 用于移动平均滤波的参数
FILTER_WINDOW_SIZE = 5  # 滤波窗口大小
distance_buffer = []    # 存储最近的距离测量值

# 不同目标的震动等级（占空比）
vibration_levels = {
    'car': 100,
    'traffic light': 70,
    'person': 40
}

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

def _make_grid(nx, ny):
    xv, yv = np.meshgrid(np.arange(ny), np.arange(nx))
    return np.stack((xv, yv), 2).reshape((-1, 2)).astype(np.float32)

def cal_outputs(outs, nl, na, model_w, model_h, anchor_grid, stride):
    row_ind = 0
    grid = [np.zeros(1)] * nl
    for i in range(nl):
        h, w = int(model_w / stride[i]), int(model_h / stride[i])
        length = int(na * h * w)
        if grid[i].shape[2:4] != (h, w):
            grid[i] = _make_grid(w, h)

        outs[row_ind:row_ind + length, 0:2] = (outs[row_ind:row_ind + length, 0:2] * 2. - 0.5 + np.tile(
            grid[i], (na, 1))) * int(stride[i])
        outs[row_ind:row_ind + length, 2:4] = (outs[row_ind:row_ind + length, 2:4] * 2) ** 2 * np.repeat(
            anchor_grid[i], h * w, axis=0)
        row_ind += length
    return outs

def post_process_opencv(outputs, model_h, model_w, img_h, img_w, thred_nms, thred_cond):
    conf = outputs[:, 4].tolist()
    c_x = outputs[:, 0] / model_w * img_w
    c_y = outputs[:, 1] / model_h * img_h
    w = outputs[:, 2] / model_w * img_w
    h = outputs[:, 3] / model_h * img_h
    p_cls = outputs[:, 5:]
    if len(p_cls.shape) == 1:
        p_cls = np.expand_dims(p_cls, 1)
    cls_id = np.argmax(p_cls, axis=1)

    p_x1 = np.expand_dims(c_x - w / 2, -1)
    p_y1 = np.expand_dims(c_y - h / 2, -1)
    p_x2 = np.expand_dims(c_x + w / 2, -1)
    p_y2 = np.expand_dims(c_y + h / 2, -1)
    areas = np.concatenate((p_x1, p_y1, p_x2, p_y2), axis=-1)

    areas = areas.tolist()
    ids = cv2.dnn.NMSBoxes(areas, conf, thred_cond, thred_nms)
    if len(ids) > 0:
        return np.array(areas)[ids], np.array(conf)[ids], cls_id[ids]
    else:
        return [], [], []

def infer_img(img0, net, model_h, model_w, nl, na, stride, anchor_grid, thred_nms=0.4, thred_cond=0.5):
    # 图像预处理
    img = cv2.resize(img0, [model_w, model_h], interpolation=cv2.INTER_AREA)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    blob = np.expand_dims(np.transpose(img, (2, 0, 1)), axis=0)

    # 模型推理
    outs = net.run(None, {net.get_inputs()[0].name: blob})[0].squeeze(axis=0)

    # 输出坐标矫正
    outs = cal_outputs(outs, nl, na, model_w, model_h, anchor_grid, stride)

    # 检测框计算
    img_h, img_w, _ = np.shape(img0)
    boxes, confs, ids = post_process_opencv(outs, model_h, model_w, img_h, img_w, thred_nms, thred_cond)

    return boxes, confs, ids

MPU_Init()
print("MPU6050 初始化完成")

# 模型加载
model_pb_path = "best.onnx"
so = ort.SessionOptions()
net = ort.InferenceSession(model_pb_path, so)

# 标签字典
dic_labels = {0: 'person',
              1: 'traffic light',
              2: 'car'}
# 中文标签映射
chinese_labels = {
    'person': '人',
    'traffic light': '交通信号灯',
    'car': '车'
}
# 各类别的置信度阈值
confidence_thresholds = {
    'person': 0.6,
    'traffic light': 0.5,
    'car': 0.8
}

# 模型参数
model_h = 320
model_w = 320
nl = 3
na = 3
stride = [8., 16., 32.]
anchors = [[10, 13, 16, 30, 33, 23], [30, 61, 62, 45, 59, 119], [116, 90, 156, 198, 373, 326]]
anchor_grid = np.asarray(anchors, dtype=np.float32).reshape(nl, -1, 2)

video = 0
cap = cv2.VideoCapture(video)
flag_det = True

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
                time.sleep(1)
            else:
                duty_cycle = 10
                pygame.mixer.music.load(obstacle_audio_path)
                pygame.mixer.music.play()
                print("前方有障碍！")
                time.sleep(1)

            # 改变 PWM 的占空比
            pwm.ChangeDutyCycle(duty_cycle)

        success, img0 = cap.read()
        if success:
            if flag_det:
                t1 = time.time()
                det_boxes, scores, ids = infer_img(img0, net, model_h, model_w, nl, na, stride, anchor_grid, thred_nms=0.4, thred_cond=0.5)
                t2 = time.time()

                for score, id in zip(scores, ids):
                    label = dic_labels[id]
                    chinese_label = chinese_labels[label]
                    threshold = confidence_thresholds[label]
                    if score >= threshold:
                        if label == 'car':
                            pygame.mixer.music.load(car_audio_path)
                            pygame.mixer.music.play()
                            print('前方有车')
                            pwm.ChangeDutyCycle(vibration_levels['car'])
                            time.sleep(3)  # 等待音频播放
                            pwm.ChangeDutyCycle(0)
                        elif label == 'person':
                            pygame.mixer.music.load(person_audio_path)
                            pygame.mixer.music.play()
                            print('前方有人')
                            pwm.ChangeDutyCycle(vibration_levels['person'])
                            time.sleep(3)  # 等待音频播放
                            pwm.ChangeDutyCycle(0)
                        elif label == 'traffic light':
                            pygame.mixer.music.load(light_audio_path)
                            pygame.mixer.music.play()
                            print('前方有交通信号灯')
                            pwm.ChangeDutyCycle(vibration_levels['traffic light'])
                            time.sleep(3)  # 等待音频播放
                            pwm.ChangeDutyCycle(0)

            cv2.imshow("video", img0)  # 显示视频流

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        time.sleep(1)  # 等待1秒再进行下一次测量

except KeyboardInterrupt:
    print("程序终止")
finally:
    # 停止 PWM
    pwm.stop()
    # 清理 GPIO 设置
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()