#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
import threading
import os
import time
import sys
import json
import queue
from vosk import Model, KaldiRecognizer
import pyaudio
import tkinter as tk
from tkinter import ttk
import subprocess

class VoiceNavigationGUI:
    def __init__(self, node):
        self.node = node
        self.gui_queue = queue.Queue()
        
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title("语音导航控制系统")
        self.root.geometry("800x600")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # 设置样式
        self.style = ttk.Style()
        self.style.configure("Title.TLabel", font=("Arial", 20, "bold"))
        self.style.configure("Subtitle.TLabel", font=("Arial", 14, "bold"))
        self.style.configure("Status.TLabel", font=("Arial", 16))
        self.style.configure("Command.TLabel", font=("Arial", 12))
        self.style.configure("Map.TFrame", background="white")
        
        # 创建主框架
        main_frame = ttk.Frame(self.root, padding=20)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 标题
        title_label = ttk.Label(main_frame, text="机器人语音导航控制系统", style="Title.TLabel")
        title_label.pack(pady=10)
        
        # 创建左右两个面板
        paned_window = ttk.PanedWindow(main_frame, orient=tk.HORIZONTAL)
        paned_window.pack(fill=tk.BOTH, expand=True, pady=20)
        
        # 左侧面板 - 状态信息
        status_frame = ttk.LabelFrame(paned_window, text="导航状态", padding=15)
        paned_window.add(status_frame, weight=1)
        
        # 当前位置
        ttk.Label(status_frame, text="当前位置:", style="Subtitle.TLabel").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.current_position_var = tk.StringVar(value="目标点1")
        ttk.Label(status_frame, textvariable=self.current_position_var, style="Status.TLabel").grid(row=0, column=1, sticky=tk.W, pady=5)
        
        # 目标位置
        ttk.Label(status_frame, text="目标位置:", style="Subtitle.TLabel").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.target_position_var = tk.StringVar(value="无")
        ttk.Label(status_frame, textvariable=self.target_position_var, style="Status.TLabel").grid(row=1, column=1, sticky=tk.W, pady=5)
        
        # 导航状态
        ttk.Label(status_frame, text="导航状态:", style="Subtitle.TLabel").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.nav_status_var = tk.StringVar(value="空闲")
        ttk.Label(status_frame, textvariable=self.nav_status_var, style="Status.TLabel").grid(row=2, column=1, sticky=tk.W, pady=5)
        
        # 语音命令
        ttk.Label(status_frame, text="最新语音命令:", style="Subtitle.TLabel").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.command_var = tk.StringVar(value="等待命令...")
        ttk.Label(status_frame, textvariable=self.command_var, style="Command.TLabel", wraplength=300).grid(row=3, column=1, sticky=tk.W, pady=5)
        
        # 手动控制按钮
        button_frame = ttk.Frame(status_frame)
        button_frame.grid(row=4, column=0, columnspan=2, pady=20)
        
        ttk.Button(button_frame, text="目标点1", command=lambda: self.send_waypoint(1)).grid(row=0, column=0, padx=5)
        ttk.Button(button_frame, text="目标点2", command=lambda: self.send_waypoint(2)).grid(row=0, column=1, padx=5)
        ttk.Button(button_frame, text="目标点3", command=lambda: self.send_waypoint(3)).grid(row=0, column=2, padx=5)
        ttk.Button(button_frame, text="目标点4", command=lambda: self.send_waypoint(4)).grid(row=0, column=3, padx=5)
        ttk.Button(button_frame, text="目标点5", command=lambda: self.send_waypoint(5)).grid(row=0, column=4, padx=5)
        
        # 右侧面板 - 地图预览
        map_frame = ttk.LabelFrame(paned_window, text="地图预览", padding=15)
        paned_window.add(map_frame, weight=1)
        
        # 创建地图画布
        self.map_canvas = tk.Canvas(map_frame, bg="white", width=300, height=300)
        self.map_canvas.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # 绘制简单地图
        self.draw_simple_map()
        
        # 日志区域
        log_frame = ttk.LabelFrame(main_frame, text="系统日志", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        self.log_text = tk.Text(log_frame, height=6, state=tk.DISABLED)
        scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # 启动GUI更新循环
        self.root.after(100, self.update_gui)
    
    def draw_simple_map(self):
        """绘制简单的地图示意图"""
        canvas = self.map_canvas
        canvas.delete("all")
        
        # 绘制边界
        canvas.create_rectangle(20, 20, 280, 280, outline="black")
        
        # 绘制目标点
        points = [
            (50, 50, "1"),   # 目标点1
            (250, 50, "2"),  # 目标点2
            (250, 250, "3"), # 目标点3
            (50, 250, "4"),  # 目标点4
            (150, 150, "5")  # 目标点5
        ]
        
        # 绘制所有目标点
        for x, y, label in points:
            color = "red" if label == self.current_position_var.get()[-1] else "blue"
            canvas.create_oval(x-10, y-10, x+10, y+10, fill=color, outline="black")
            canvas.create_text(x, y, text=f"目标点{label}", font=("Arial", 10, "bold"))
        
        # 绘制机器人当前位置
        current_pos = {
            "1": (50, 50),
            "2": (250, 50),
            "3": (250, 250),
            "4": (50, 250),
            "5": (150, 150)
        }.get(self.current_position_var.get()[-1], (150, 150))
        
        # 绘制机器人
        x, y = current_pos
        canvas.create_rectangle(x-8, y-15, x+8, y+5, fill="green", outline="black")
        canvas.create_oval(x-8, y+5, x-3, y+10, fill="black")  # 左轮
        canvas.create_oval(x+3, y+5, x+8, y+10, fill="black")  # 右轮
        
        # 如果正在导航，绘制路径
        if self.nav_status_var.get() != "空闲":
            target_pos = {
                "1": (50, 50),
                "2": (250, 50),
                "3": (250, 250),
                "4": (50, 250),
                "5": (150, 150)
            }.get(self.target_position_var.get()[-1], (150, 150))
            
            if target_pos:
                canvas.create_line(x, y, target_pos[0], target_pos[1], arrow=tk.LAST, fill="green", width=2, dash=(5, 2))
    
    def send_waypoint(self, waypoint):
        """手动发送航点命令"""
        self.node.process_voice_command(f"目标点{waypoint}")
    
    def update_gui(self):
        """更新GUI内容"""
        try:
            while True:
                # 从队列中获取更新消息
                msg_type, data = self.gui_queue.get_nowait()
                
                if msg_type == "command":
                    self.command_var.set(data)
                elif msg_type == "position":
                    self.current_position_var.set(data)
                elif msg_type == "target":
                    self.target_position_var.set(data)
                elif msg_type == "status":
                    self.nav_status_var.set(data)
                elif msg_type == "log":
                    self.log_text.config(state=tk.NORMAL)
                    self.log_text.insert(tk.END, data + "\n")
                    self.log_text.see(tk.END)
                    self.log_text.config(state=tk.DISABLED)
                
                # 重绘地图
                self.draw_simple_map()
                
        except queue.Empty:
            pass
        
        # 继续调度下一次更新
        self.root.after(100, self.update_gui)
    
    def on_close(self):
        """关闭窗口时的处理"""
        self.root.destroy()
        self.node.get_logger().info("GUI关闭，程序退出")
        self.node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    def run(self):
        """运行GUI主循环"""
        self.root.mainloop()

class VoiceNavigationNode(Node):
    def __init__(self, gui_queue):
        super().__init__('voice_navigation_node')
        self.gui_queue = gui_queue
        
        # 添加日志到GUI
        self.gui_queue.put(("log", "系统启动中..."))
        
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
            self.gui_queue.put(("log", "使用中文语音"))
        else:
            self.gui_queue.put(("log", "未找到中文语音，使用默认语音"))
        
        # 初始化Vosk离线语音识别模型
        self.model = None
        model_path = os.path.expanduser("~/vosk-model-cn-0.22")  # 使用小型中文模型
        
        if not os.path.exists(model_path):
            self.gui_queue.put(("log", f"未找到Vosk模型，请按照说明下载并解压到: {model_path}"))
            self.speak("语音识别模型未找到，请按照说明下载模型")
            # 尝试使用在线识别作为备选
            self.use_online_recognition = True
            self.gui_queue.put(("log", "将尝试使用在线识别作为备选方案"))
        else:
            try:
                self.model = Model(model_path)
                self.gui_queue.put(("log", "Vosk中文离线模型加载成功"))
                self.use_online_recognition = False
            except Exception as e:
                self.gui_queue.put(("log", f"加载Vosk模型失败: {str(e)}"))
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
            self.gui_queue.put(("log", "无法找到可用的麦克风设备！"))
            self.speak("麦克风初始化失败，无法进行语音识别")
            return
        
        # 初始化Vosk识别器
        if self.model:
            self.recognizer = KaldiRecognizer(self.model, 16000)
        
        # 启动语音识别线程
        self.voice_thread = threading.Thread(target=self.voice_listener)
        self.voice_thread.daemon = True
        self.voice_thread.start()
        
        self.gui_queue.put(("log", "语音导航系统已启动，请说出目标点命令"))
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
            
            self.gui_queue.put(("log", f"可用的麦克风设备: {mic_list}"))
            
            # 如果没有设备，直接返回
            if not mic_list:
                self.gui_queue.put(("log", "未找到可用的麦克风设备"))
                self.mic_index = None
                return
            
            # 优先选择名称中包含'pulse'的设备
            pulse_devices = [item for item in mic_list if 'pulse' in item[1].lower()]
            if pulse_devices:
                self.mic_index = pulse_devices[0][0]
                self.gui_queue.put(("log", f"选择Pulse麦克风设备: {pulse_devices[0][1]}"))
                return
            
            # 其次选择名称中包含'default'的设备
            default_devices = [item for item in mic_list if 'default' in item[1].lower()]
            if default_devices:
                self.mic_index = default_devices[0][0]
                self.gui_queue.put(("log", f"选择默认麦克风设备: {default_devices[0][1]}"))
                return
            
            # 最后选择第一个可用的输入设备
            self.mic_index = mic_list[0][0]
            self.gui_queue.put(("log", f"使用第一个可用麦克风设备: {mic_list[0][1]}"))
        
        except Exception as e:
            self.gui_queue.put(("log", f"麦克风初始化失败: {str(e)}"))
            self.mic_index = None
    
    def speak(self, text):
        """将文本转换为语音输出"""
        self.gui_queue.put(("log", f"[语音反馈]: {text}"))
        try:
            self.engine.say(text)
            self.engine.runAndWait()
        except Exception as e:
            self.gui_queue.put(("log", f"语音输出失败: {str(e)}"))
    
    def recognize_with_vosk(self, audio_data):
        """使用Vosk进行离线语音识别"""
        if self.recognizer:
            if self.recognizer.AcceptWaveform(audio_data):
                result = self.recognizer.Result()
                result_json = json.loads(result)
                text = result_json.get('text', '').strip()
                if text:
                    self.gui_queue.put(("log", f"离线识别结果: {text}"))
                    self.gui_queue.put(("command", text))
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
                self.gui_queue.put(("log", f"在线识别结果: {text}"))
                self.gui_queue.put(("command", text))
                return text
            except sr.UnknownValueError:
                self.gui_queue.put(("log", "在线识别无法理解语音"))
            except sr.RequestError as e:
                self.gui_queue.put(("log", f"在线识别请求错误: {str(e)}"))
            except Exception as e:
                self.gui_queue.put(("log", f"在线识别异常: {str(e)}"))
            
            # 尝试英文识别
            try:
                text = recognizer.recognize_google(audio)
                self.gui_queue.put(("log", f"在线识别结果: {text}"))
                self.gui_queue.put(("command", text))
                return text
            except:
                pass
        
        except ImportError:
            self.gui_queue.put(("log", "未安装SpeechRecognition库，无法使用在线识别"))
        except Exception as e:
            self.gui_queue.put(("log", f"在线识别失败: {str(e)}"))
        
        return None
    
    def voice_listener(self):
        """持续监听语音命令"""
        if self.mic_index is None:
            self.gui_queue.put(("log", "麦克风不可用，语音监听无法启动"))
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
            
            self.gui_queue.put(("log", "开始监听语音..."))
            self.gui_queue.put(("log", "请说出目标点命令（如'目标点1'、'点二'等）"))
            
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
                        self.gui_queue.put(("log", "输入溢出，跳过部分音频"))
                    else:
                        self.gui_queue.put(("log", f"音频流错误: {str(e)}"))
                except Exception as e:
                    self.gui_queue.put(("log", f"语音监听错误: {str(e)}"))
                    time.sleep(0.5)
        except Exception as e:
            self.gui_queue.put(("log", f"无法打开音频流: {str(e)}"))
    
    def process_voice_command(self, text):
        """处理识别到的语音命令"""
        # 转换为小写以简化匹配
        text = text.lower()
        
        # 更新GUI命令显示
        self.gui_queue.put(("command", text))
        
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
            self.gui_queue.put(("target", f"目标点{waypoint}"))
            self.gui_queue.put(("status", "正在前往..."))
            self.speak(f"收到命令，导航到目标点{waypoint}")
            self.navigate_to_waypoint(waypoint)
        else:
            self.speak("未识别到有效命令，请说目标点1到5")
            self.gui_queue.put(("log", f"未识别命令: {text}"))
    
    def navigate_to_waypoint(self, waypoint_id):
        """发布导航命令"""
        msg = String()
        msg.data = waypoint_id
        self.navigation_pub.publish(msg)
        self.gui_queue.put(("log", f"发布导航命令: 目标点{waypoint_id}"))
    
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
    
    # 创建GUI队列（用于节点向GUI传递消息）
    gui_queue = queue.Queue()
    
    # 创建ROS节点（在单独的线程中运行）
    node = VoiceNavigationNode(gui_queue)
    
    # 创建GUI（在主线程中）
    gui = VoiceNavigationGUI(node)
    
    # 将队列传递给GUI
    gui.gui_queue = gui_queue
    
    # 启动ROS节点线程
    def spin_node():
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    
    node_thread = threading.Thread(target=spin_node)
    node_thread.daemon = True
    node_thread.start()
    
    # 检查麦克风是否初始化成功
    if node.mic_index is None:
        gui_queue.put(("log", "由于麦克风初始化失败，程序将退出"))
        time.sleep(2)
        gui.root.destroy()
        return
    
    # 启动GUI主循环（在主线程中）
    gui.run()

if __name__ == '__main__':
    main()