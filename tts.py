# #!/usr/bin/env python3
# import rospy
# from sound_play.msg import SoundRequest
# from sound_play.libsoundplay import SoundClient

# if __name__ == '__main__':
#     rospy.init_node('tts_publisher')
#     soundclient = SoundClient()  # 创建音频客户端
#     rospy.sleep(1)  # 等待客户端连接

#     # 输出指定文本（英文示例，可替换为其他语言）
#     soundclient.say('Hello, this is a test speech from ROS.', 'voice_kal_diphone')  # 指定语音模型
#     # soundclient.say('你好，这是一个测试。', 'voice_ekho_zh')
#     rospy.spin()  # 保持节点运行


#!/usr/bin/env python3
import rospy
import pyttsx3

if __name__ == '__main__':
    rospy.init_node('chinese_tts_publisher')
    engine = pyttsx3.init()  # 初始化引擎

    # 设置中文语音（Mandarin）
    voices = engine.getProperty('voices')
    for voice in voices:
        if 'mandarin' in voice.id.lower() or 'zh' in voice.id.lower():
            engine.setProperty('voice', voice.id)
            break
    engine.setProperty('rate', 120)  # 语速（120-200正常）
    engine.setProperty('volume', 0.5)  # 音量（0-1）

    # 输出中文文本
    text = '车牌号为浙Ｆ 65L05'
    engine.say(text)
    engine.runAndWait()  # 阻塞播放，直到完成

    rospy.spin()  # 保持节点运行，可扩展为话题订阅