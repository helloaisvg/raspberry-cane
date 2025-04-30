import pygame
import time

# 初始化pygame的mixer模块
pygame.mixer.init()

# 加载音频文件（路径根据实际情况修改）
pygame.mixer.music.load("/home/pi/test/one.mp3")

# 播放音频
pygame.mixer.music.play()

# 等待音频播放结束
while pygame.mixer.music.get_busy():  # 音频正在播放
    time.sleep(1)

print("音频播放完毕！")
