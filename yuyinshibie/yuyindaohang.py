#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
import threading
import os
import re
import time
import sys
import json
from vosk import Model, KaldiRecognizer
import pyaudio

class VoiceNavigationNode(Node):
    def __init__(self):
        super().__init__('voice_navigation_node')
        
        # 创建导航发布者
        self.navigation_pub = self.create_publisher(
            String,
            '/waterplus/navi_waypoint',
            10
        )
        
        # 语音引擎初始化
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)    # 语速
        self.engine.setProperty('volume', 0.9)  # 音量
        
        # 选择中文语音（如果有）
        voices = self.engine.getProperty('voices')
        chinese_voice = next((v for v in voices if 'chinese' in v.id.lower() or 'zh' in v.id.lower()), None)
        if chinese_voice:
            self.engine.setProperty('voice', chinese_voice.id)
            self.get_logger().info("使用中文语音")
        else:
            self.get_logger().info("未找到中文语音，使用默认语音")
        
        # 初始化Vosk离线语音识别模型
        self.model = None
        model_path = os.path.expanduser("~/vosk-model-cn-0.22")  # 使用小型中文模型
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"未找到Vosk模型，请按照说明下载并解压到: {model_path}")
            self.speak("语音识别模型未找到，请按照说明下载模型")
            # 尝试使用在线识别作为备选
            self.use_online_recognition = True
            self.get_logger().warning("将尝试使用在线识别作为备选方案")
        else:
            try:
                self.model = Model(model_path)
                self.get_logger().info("Vosk中文离线模型加载成功")
                self.use_online_recognition = False
            except Exception as e:
                self.get_logger().error(f"加载Vosk模型失败: {str(e)}")
                self.use_online_recognition = True
        
        # 初始化音频输入
        self.audio = pyaudio.PyAudio()
        self.recognizer = None
        self.stream = None
        self.listening = True
        self.mic_index = None
        
        # 查找可用的麦克风设备
        self.find_working_microphone()
        
        if self.mic_index is None:
            self.get_logger().error("无法找到可用的麦克风设备！")
            self.speak("麦克风初始化失败，无法进行语音识别")
            return
        
        # 初始化Vosk识别器
        if self.model:
            self.recognizer = KaldiRecognizer(self.model, 16000)
        
        # 启动语音识别线程
        self.voice_thread = threading.Thread(target=self.voice_listener)
        self.voice_thread.daemon = True
        self.voice_thread.start()
        
        self.get_logger().info("语音导航系统已启动，请说出目标点命令")
        self.speak("语音导航系统已启动，请说出目标点命令")
    
    def find_working_microphone(self):
        """尝试找到可用的麦克风设备"""
        try:
            # 获取所有可用设备
            device_count = self.audio.get_device_count()
            mic_list = []
            
            for i in range(device_count):
                try:
                    device_info = self.audio.get_device_info_by_index(i)
                    if device_info.get('maxInputChannels', 0) > 0:
                        mic_list.append((i, device_info['name']))
                except:
                    continue
            
            self.get_logger().info(f"可用的麦克风设备: {mic_list}")
            
            # 如果没有设备，直接返回
            if not mic_list:
                self.get_logger().warning("未找到可用的麦克风设备")
                self.mic_index = None
                return
            
            # 优先选择名称中包含'pulse'的设备
            pulse_devices = [item for item in mic_list if 'pulse' in item[1].lower()]
            if pulse_devices:
                self.mic_index = pulse_devices[0][0]
                self.get_logger().info(f"选择Pulse麦克风设备: {pulse_devices[0][1]}")
                return
            
            # 其次选择名称中包含'default'的设备
            default_devices = [item for item in mic_list if 'default' in item[1].lower()]
            if default_devices:
                self.mic_index = default_devices[0][0]
                self.get_logger().info(f"选择默认麦克风设备: {default_devices[0][1]}")
                return
            
            # 最后选择第一个可用的输入设备
            self.mic_index = mic_list[0][0]
            self.get_logger().info(f"使用第一个可用麦克风设备: {mic_list[0][1]}")
        
        except Exception as e:
            self.get_logger().error(f"麦克风初始化失败: {str(e)}")
            self.mic_index = None
    
    def speak(self, text):
        """将文本转换为语音输出"""
        self.get_logger().info(f"[语音反馈]: {text}")
        try:
            self.engine.say(text)
            self.engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f"语音输出失败: {str(e)}")
    
    def recognize_with_vosk(self, audio_data):
        """使用Vosk进行离线语音识别"""
        if self.recognizer:
            if self.recognizer.AcceptWaveform(audio_data):
                result = self.recognizer.Result()
                result_json = json.loads(result)
                text = result_json.get('text', '').strip()
                if text:
                    self.get_logger().info(f"离线识别结果: {text}")
                    return text
        return None
    
    def recognize_online(self, audio_data):
        """使用在线识别作为备选方案"""
        try:
            import speech_recognition as sr
            
            # 创建音频数据对象
            audio = sr.AudioData(audio_data, 16000, 2)
            recognizer = sr.Recognizer()
            
            # 尝试中文识别
            try:
                text = recognizer.recognize_google(audio, language='zh-CN')
                self.get_logger().info(f"在线识别结果: {text}")
                return text
            except sr.UnknownValueError:
                self.get_logger().warn("在线识别无法理解语音")
            except sr.RequestError as e:
                self.get_logger().error(f"在线识别请求错误: {str(e)}")
            except Exception as e:
                self.get_logger().error(f"在线识别异常: {str(e)}")
            
            # 尝试英文识别
            try:
                text = recognizer.recognize_google(audio)
                self.get_logger().info(f"在线识别结果: {text}")
                return text
            except:
                pass
        
        except ImportError:
            self.get_logger().error("未安装SpeechRecognition库，无法使用在线识别")
        except Exception as e:
            self.get_logger().error(f"在线识别失败: {str(e)}")
        
        return None
    
    def voice_listener(self):
        """持续监听语音命令"""
        if self.mic_index is None:
            self.get_logger().error("麦克风不可用，语音监听无法启动")
            return
        
        try:
            # 打开音频流 - 增加缓冲区大小减少ALSA错误
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=16000,
                input=True,
                input_device_index=self.mic_index,
                frames_per_buffer=4096
            )
            
            self.get_logger().info("开始监听语音...")
            self.get_logger().info("请说出目标点命令（如'目标点1'、'点二'等）")
            
            # 对于Vosk模型，使用主动轮询方式
            while self.listening and rclpy.ok():
                try:
                    # 读取音频数据
                    data = self.stream.read(4096, exception_on_overflow=False)
                    
                    # 使用Vosk进行离线识别
                    text = None
                    if self.model and self.recognizer:
                        text = self.recognize_with_vosk(data)
                    
                    # 如果离线识别失败且启用了在线识别
                    if not text and self.use_online_recognition:
                        text = self.recognize_online(data)
                    
                    if text:
                        self.process_voice_command(text)
                
                except OSError as e:
                    if "Input overflowed" in str(e):
                        self.get_logger().warn("输入溢出，跳过部分音频")
                    else:
                        self.get_logger().error(f"音频流错误: {str(e)}")
                except Exception as e:
                    self.get_logger().error(f"语音监听错误: {str(e)}")
                    time.sleep(0.5)
        except Exception as e:
            self.get_logger().error(f"无法打开音频流: {str(e)}")
    
    def process_voice_command(self, text):
        """处理识别到的语音命令"""
        # 转换为小写以简化匹配
        text = text.lower()
        
        # 尝试匹配目标点命令
        waypoint = None
        if "目标点1" in text or "点一" in text or "one" in text or "一" in text:
            waypoint = "1"
        elif "目标点2" in text or "点二" in text or "two" in text or "二" in text or "两" in text:
            waypoint = "2"
        elif "目标点3" in text or "点三" in text or "three" in text or "三" in text:
            waypoint = "3"
        elif "目标点4" in text or "点四" in text or "four" in text or "四" in text:
            waypoint = "4"
        elif "目标点5" in text or "点五" in text or "five" in text or "五" in text:
            waypoint = "5"
        
        if waypoint:
            self.speak(f"收到命令，导航到目标点{waypoint}")
            self.navigate_to_waypoint(waypoint)
        else:
            self.speak("未识别到有效命令，请说目标点1到5")
            self.get_logger().info(f"未识别命令: {text}")
    
    def navigate_to_waypoint(self, waypoint_id):
        """发布导航命令"""
        msg = String()
        msg.data = waypoint_id
        self.navigation_pub.publish(msg)
        self.get_logger().info(f"发布导航命令: 目标点{waypoint_id}")
    
    def destroy_node(self):
        """节点销毁时的清理"""
        self.listening = False
        if hasattr(self, 'voice_thread') and self.voice_thread.is_alive():
            self.voice_thread.join(timeout=1.0)
        
        if hasattr(self, 'stream') and self.stream:
            try:
                self.stream.stop_stream()
                self.stream.close()
            except:
                pass
        
        if hasattr(self, 'audio') and self.audio:
            try:
                self.audio.terminate()
            except:
                pass
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = VoiceNavigationNode()
        if node.mic_index is not None:  # 只有在麦克风初始化成功时才启动主循环
            rclpy.spin(node)
        else:
            node.get_logger().error("由于麦克风初始化失败，程序将退出")
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"程序启动失败: {str(e)}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()