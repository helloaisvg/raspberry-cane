import serial
import pynmea2
import time

ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=9600,
    timeout=1
)

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
                    print(f"解析错误: {e}")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("程序终止")
finally:
    ser.close()