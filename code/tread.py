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
import threading
import queue

# GPS串口设置
ser = serial.Serial(port='/dev/ttyS0', baudrate=9600, timeout=1)

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

# 定义GPIO引脚
MOTOR_PIN = 18
TRIG = 23
ECHO = 24

# GPIO设置
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(MOTOR_PIN, GPIO.OUT)
pwm = GPIO.PWM(MOTOR_PIN, 100)  # 初始化PWM频率为100Hz
pwm.start(0)  # 初始占空比为0

# 初始化pygame音频模块
pygame.mixer.init()

# 加载音频文件
audio_files = {
    "fall": "/home/pi/code/two.mp3",
    "obstacle": "/home/pi/code/one.mp3",
    "person": "/home/pi/code/person.mp3",
    "car": "/home/pi/code/car.mp3",
    "traffic light": "/home/pi/code/light.mp3"
}

# 加载所有音频文件到pygame
for key, path in audio_files.items():
    pygame.mixer.music.load(path)

# 移动平均滤波参数
FILTER_WINDOW_SIZE = 5
distance_buffer = []

def MPU_Init():
    # 初始化MPU6050模块
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 0)
    bus.write_byte_data(Device_Address, ACCEL_CONFIG, 0)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    # 读取高低字节数据并合并
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)
    value = (high << 8) | low
    return value - 65536 if value > 32768 else value

def detect_fall(Ax, Ay, Az, Gx, Gy, Gz):
    # 检测是否摔倒
    accel_magnitude = math.sqrt(Ax**2 + Ay**2 + Az**2)
    gyro_magnitude = math.sqrt(Gx**2 + Gy**2 + Gz**2)
    return accel_magnitude > 2 and gyro_magnitude > 150

def measure_distance():
    # 发送超声波脉冲并计算距离
    GPIO.output(TRIG, False)
    time.sleep(0.01)  # 减少等待时间
    GPIO.output(TRIG, True)
    time.sleep(0.000001)
    GPIO.output(TRIG, False)

    pulse_start, pulse_end = time.time(), time.time()
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    distance = (pulse_end - pulse_start) * 34300 / 2  # 计算距离
    return distance

def update_distance_buffer(distance):
    # 更新距离缓冲区并计算平均值
    distance_buffer.append(distance)
    if len(distance_buffer) > FILTER_WINDOW_SIZE:
        distance_buffer.pop(0)
    return sum(distance_buffer) / len(distance_buffer)

def play_audio_async(audio_type):
    # 异步播放音频
    def play():
        pygame.mixer.music.load(audio_files[audio_type])
        pygame.mixer.music.play()
    audio_thread = threading.Thread(target=play)
    audio_thread.start()

def _make_grid(nx, ny):
    xv, yv = np.meshgrid(np.arange(ny), np.arange(nx))
    return np.stack((xv, yv), 2).reshape((-1, 2)).astype(np.float32)

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

MPU_Init()
print("MPU6050初始化完成")

# 加载模型
model_pb_path = "best.onnx"
so = ort.SessionOptions()
net = ort.InferenceSession(model_pb_path, so)

# 标签映射
dic_labels = {0: 'person', 1: 'traffic light', 2: 'car'}
chinese_labels = {'person': '人', 'traffic light': '交通信号灯', 'car': '车'}
confidence_thresholds = {'person': 0.7, 'traffic light': 0.6, 'car': 0.8}

# 模型参数
model_h, model_w = 320, 320
nl, na = 3, 3
stride = [8., 16., 32.]
anchors = [[10, 13, 16, 30, 33, 23], [30, 61, 62, 45, 59, 119], [116, 90, 156, 198, 373, 326]]
anchor_grid = np.asarray(anchors, dtype=np.float32).reshape(nl, -1, 2)

cap = cv2.VideoCapture(0)
flag_det = True

# 新增帧计数器和检测间隔
frame_count = 0
detect_interval = 5  # 每隔7帧进行一次检测，可根据实际情况调整

# 线程队列
frame_queue = queue.Queue(maxsize=1)
detection_queue = queue.Queue(maxsize=1)

def object_detection_thread(net, model_h, model_w, nl, na, stride, anchor_grid):
    while True:
        if not frame_queue.empty():
            img0 = frame_queue.get()
            det_boxes, scores, ids = infer_img(img0, net, model_h, model_w, nl, na, stride, anchor_grid)
            detection_queue.put((img0, det_boxes, scores, ids))
        time.sleep(0.5)

# 启动目标检测线程
detection_thread = threading.Thread(target=object_detection_thread, args=(net, model_h, model_w, nl, na, stride, anchor_grid))
detection_thread.daemon = True
detection_thread.start()

try:
    while True:
        # 读取GPS数据
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            if line.startswith('$'):
                try:
                    msg = pynmea2.parse(line)
                    if isinstance(msg, pynmea2.GGA):
                        print(f"纬度: {msg.latitude}, 经度: {msg.longitude}")
                except pynmea2.ParseError as e:
                    print(f"解析错误,定位失败: {e}")

        # 读取加速度计和陀螺仪数据
        acc_x, acc_y, acc_z = (read_raw_data(ACCEL_XOUT_H + i) for i in range(0, 6, 2))
        gyro_x, gyro_y, gyro_z = (read_raw_data(GYRO_XOUT_H + i) for i in range(0, 6, 2))

        # 标准化加速度计和陀螺仪值
        Ax, Ay, Az = acc_x / 16384.0, acc_y / 16384.0, acc_z / 16384.0
        Gx, Gy, Gz = gyro_x / 131.0, gyro_y / 131.0, gyro_z / 131.0

        # 检测摔倒
        if detect_fall(Ax, Ay, Az, Gx, Gy, Gz):
            print("检测到摔倒！")
            pwm.ChangeDutyCycle(100)
            play_audio_async("fall")
            time.sleep(3)
            pwm.ChangeDutyCycle(0)
        else:
            # 测量距离并更新缓冲区
            distance = measure_distance()
            average_distance = update_distance_buffer(distance)

            print("滤波后距离: %.2f 厘米" % average_distance)

            # 根据平均距离调整电机占空比
            if average_distance > 60:
                pwm.ChangeDutyCycle(0)
            elif average_distance > 0:
                duty_cycle = int((60 - average_distance) / 60 * 100)
                pwm.ChangeDutyCycle(duty_cycle)
                play_audio_async("obstacle")
                print("前方有障碍！")
                time.sleep(0.5)
        # 读取视频流
        success, img0 = cap.read()
        if success and flag_det:
            frame_count += 1
            if frame_count % detect_interval == 0:
                if not frame_queue.full():
                    frame_queue.put(img0)

            if not detection_queue.empty():
                img0, det_boxes, scores, ids = detection_queue.get()
                for score, id in zip(scores, ids):
                    label = dic_labels[id]
                    if score >= confidence_thresholds[label]:
                        play_audio_async(label)  # 播放相应音频
                        pwm.ChangeDutyCycle(80)
                        print(f"前方有{chinese_labels[label]}")
                        time.sleep(1.5)
                        pwm.ChangeDutyCycle(0)

except KeyboardInterrupt:
    print("程序终止")
finally:
    pwm.stop()  # 停止PWM
    GPIO.cleanup()  # 清理GPIO设置
    cap.release()  # 释放摄像头
    cv2.destroyAllWindows()  # 关闭所有OpenCV窗口