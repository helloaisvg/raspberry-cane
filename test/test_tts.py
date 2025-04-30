import pyttsx3

engine = pyttsx3.init()
voices = engine.getProperty('voices')
# 选择中文语音
for voice in voices:
    if 'zh' in voice.languages:
        engine.setProperty('voice', voice.id)
        break

text = "你好，欢迎使用树莓派"
engine.say(text)
engine.runAndWait()