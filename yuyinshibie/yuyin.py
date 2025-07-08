import speech_recognition as sr
import pyttsx3
import threading

# 初始化语音引擎
engine = pyttsx3.init()

def speak(text):
    """将文本转换为语音输出"""
    print(f"\n[系统]: {text}")
    engine.say(text)
    engine.runAndWait()

def recognize_speech():
    """识别麦克风输入的语音并转换为文本"""
    recognizer = sr.Recognizer()
    
    with sr.Microphone() as source:
        print("\n请开始说话（5秒超时）...")
        recognizer.adjust_for_ambient_noise(source)  # 环境降噪
        try:
            audio = recognizer.listen(source, timeout=5)
            print("识别中...")
            
            # 使用离线识别引擎
            text = recognizer.recognize_sphinx(audio)
            print(f"\n[识别结果]: {text}")
            return text
            
        except sr.WaitTimeoutError:
            print("等待超时，未检测到语音")
            return ""
        except sr.UnknownValueError:
            print("无法识别语音")
            return ""
        except Exception as e:
            print(f"发生错误: {str(e)}")
            return ""

def main():
    # 设置语音参数
    engine.setProperty('rate', 150)    # 语速
    engine.setProperty('volume', 0.9)  # 音量
    
    # 获取可用语音
    voices = engine.getProperty('voices')
    if voices:
        # 选择中文语音（若有）或英文语音
        chinese_voice = next((v for v in voices if 'chinese' in v.id.lower()), None)
        engine.setProperty('voice', chinese_voice.id if chinese_voice else voices[0].id)

    print("="*50)
    print("语音交互系统已启动")
    print("1 - 语音转文字")
    print("2 - 文字转语音")
    print("3 - 退出程序")
    print("="*50)

    while True:
        choice = input("\n请选择操作 (1/2/3): ").strip()
        
        if choice == '1':
            # 在新线程中运行语音识别
            recognition_thread = threading.Thread(target=lambda: speak(recognize_speech()))
            recognition_thread.start()
            recognition_thread.join()
            
        elif choice == '2':
            text = input("请输入要朗读的文本: ").strip()
            if text:
                # 在新线程中运行语音合成
                tts_thread = threading.Thread(target=speak, args=(text,))
                tts_thread.start()
                tts_thread.join()
            else:
                print("输入不能为空！")
                
        elif choice == '3':
            print("程序已退出")
            break
            
        else:
            print("无效选择，请重新输入")

if __name__ == "__main__":
    main()