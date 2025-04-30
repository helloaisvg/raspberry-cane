# -*- coding: utf-8 -*
import serial
import time

ser = serial.Serial("/dev/ttyS0", 9600)
try:
    while True:
        try:
            temp = ser.readline()
            temp = temp.decode('utf-8').strip()  # 添加strip()去除多余的空白字符
            
            if temp.startswith('$GNGGA'):
                temp = temp.split(',')
                latitude = temp[2]
                longitude = temp[4]
                flag = temp[6]
                satellite = temp[7]
                if latitude != '' and longitude != '':
                    # 对经纬度进行转换
                    try:
                        latitude = int(latitude[0:2]) + (float(latitude[2:8]) / 60)
                        longitude = int(longitude[0:3]) + (float(longitude[3:9]) / 60)
                        if flag == '1':
                            print('当前坐标：%s,%s' % (latitude, longitude))
                            print('卫星数量：%s' % int(satellite))
                            print('定位时间：%s\n' % time.strftime('%Y-%m-%d %H:%M:%S'))
                        else:
                            print('定位数据无效(%s)' % time.strftime('%Y-%m-%d %H:%M:%S'))
                    except (ValueError, IndexError) as e:
                        print('数据转换错误：', e)
                else:
                    print('定位失败(%s)' % time.strftime('%Y-%m-%d %H:%M:%S'))
        except UnicodeDecodeError as e:
            print('解码错误：', e)
        except Exception as e:
            print('未知错误：', e)
            
except KeyboardInterrupt:
    print("\n程序被用户中断")
except serial.SerialException as e:
    print('串口错误：', e)
finally:
    ser.close()
    print("串口已关闭")
