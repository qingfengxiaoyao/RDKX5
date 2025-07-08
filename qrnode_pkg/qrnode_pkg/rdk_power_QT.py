#!/usr/bin/env python3
import sys
import cv2
import torch
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, QVBoxLayout, QHBoxLayout, 
                            QListWidget, QPushButton, QSplitter, QStatusBar, QFileDialog, QMessageBox)
from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QFont
import os
envpath = '/usr/local/lib/python3.10/dist-packages/cv2/qt/plugins/platforms'
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = envpath

class YOLODetector:
    def __init__(self, model_path="/home/sunrise/CHEN/power.pt", conf_thres=0.5):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        try:
            self.model = YOLO(model_path)
            self.model.to(self.device)
            self.conf_thres = conf_thres
            print(f"Successfully loaded YOLO model from {model_path} on {self.device}")
        except Exception as e:
            print(f"Failed to load YOLO model: {str(e)}", file=sys.stderr)
            sys.exit(1)

    def detect(self, frame):
        try:
            if frame is None or frame.size == 0:
                print("Empty frame received for detection", file=sys.stderr)
                return []
            
            results = self.model(frame[:, :, ::-1], conf=self.conf_thres, verbose=False)
            detections = []
            
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    conf = box.conf[0].item()
                    cls = int(box.cls[0].item())
                    label = self.model.names[cls]
                    detections.append({
                        'bbox': (x1, y1, x2, y2),
                        'label': label,
                        'confidence': conf,
                        'class_id': cls
                    })
            
            return detections
        
        except Exception as e:
            print(f"Detection error: {str(e)}", file=sys.stderr)
            return []

class ImageProcessor(Node):
    def __init__(self, detector, gui_update_callback):
        super().__init__("yolo_image_processor")
        self.bridge = CvBridge()
        self.detector = detector
        self.gui_update_callback = gui_update_callback
        self.frame_count = 0
        self.start_time = self.get_clock().now()
        self.last_valid_image = None
        self.last_detections = []
        
        # 创建图像订阅者
        self.subscription = self.create_subscription(
            Image,
            "/ascamera_hp60c/camera_publisher/rgb0/image",
            self.image_callback,
            10
        )
        self.get_logger().info("ROS2 image processor initialized")

    def image_callback(self, msg):
        try:
            # 转换ROS图像消息为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检查图像是否有效
            if cv_image is None or cv_image.size == 0:
                print("Received empty or invalid image", file=sys.stderr)
                return
            
            # 保存最后一次有效图像
            self.last_valid_image = cv_image
            
            # 执行目标检测
            detections = self.detector.detect(cv_image)
            self.last_detections = detections
            
            # 计算FPS
            self.frame_count += 1
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
            
            # 通知GUI更新
            if self.gui_update_callback:
                self.gui_update_callback(cv_image, detections, fps)
                
        except Exception as e:
            self.get_logger().error(f"Processing error: {str(e)}")
            # 通知GUI更新最后有效图像
            if self.last_valid_image is not None and self.gui_update_callback:
                self.gui_update_callback(self.last_valid_image, self.last_detections, 0)

class DetectionViewer(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setText("Waiting for image...")
        self.setMinimumSize(640, 480)
        self.scale_factor = 1.0
        self.pan_offset = [0, 0]
        self.dragging = False
        self.last_pos = None
        self.detections = []
        self.fps = 0
        self.original_image = None
        
    def display_image(self, image, detections, fps):
        """显示图像和检测结果"""
        if image is None or image.size == 0:
            return
            
        self.original_image = image.copy()
        self.detections = detections
        self.fps = fps
        
        # 在图像上绘制检测结果
        display_image = self.draw_detections(image.copy(), detections)
        
        # 应用缩放和平移
        scaled_image = self.apply_zoom_and_pan(display_image)
        
        # 转换为Qt图像格式
        height, width, channel = scaled_image.shape
        bytes_per_line = 3 * width
        q_img = QImage(scaled_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        
        # 显示图像
        self.setPixmap(QPixmap.fromImage(q_img))
        
    def apply_zoom_and_pan(self, image):
        """应用缩放和平移变换"""
        if image is None or image.size == 0:
            return image
            
        # 1. 应用缩放
        if self.scale_factor != 1.0:
            h, w = image.shape[:2]
            new_w = int(w * self.scale_factor)
            new_h = int(h * self.scale_factor)
            
            # 选择适当的插值方法
            interpolation = cv2.INTER_AREA if self.scale_factor < 1.0 else cv2.INTER_LINEAR
            resized = cv2.resize(image, (new_w, new_h), interpolation=interpolation)
        else:
            resized = image
        
        # 2. 应用平移
        h, w = resized.shape[:2]
        canvas = np.zeros((h, w, 3), dtype=np.uint8)
        
        # 计算平移位置
        x_start = max(0, min(w - w, self.pan_offset[0]))
        y_start = max(0, min(h - h, self.pan_offset[1]))
        
        # 将图像放置到画布上
        canvas[:h, :w] = resized
        
        return canvas
    
    def draw_detections(self, image, detections):
        """在图像上绘制检测结果"""
        if image is None or image.size == 0:
            return image
            
        # 绘制检测框
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            label = detection['label']
            conf = detection['confidence']
            
            # 绘制矩形框
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 绘制标签背景
            text = f"{label} {conf:.2f}"
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.rectangle(image, (x1, y1 - text_height - 10), (x1 + text_width, y1), (0, 255, 0), -1)
            
            # 绘制标签文本
            cv2.putText(image, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        
        # 绘制FPS
        cv2.putText(image, f"FPS: {self.fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 绘制缩放比例
        cv2.putText(image, f"Zoom: {self.scale_factor:.1f}x", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        return image
    
    def wheelEvent(self, event):
        """鼠标滚轮缩放"""
        zoom_in = event.angleDelta().y() > 0
        if zoom_in:
            self.scale_factor = min(3.0, self.scale_factor + 0.1)
        else:
            self.scale_factor = max(0.5, self.scale_factor - 0.1)
        
        # 更新显示
        if self.original_image is not None:
            self.display_image(self.original_image, self.detections, self.fps)
    
    def mousePressEvent(self, event):
        """鼠标按下事件"""
        if event.button() == Qt.LeftButton:
            self.dragging = True
            self.last_pos = event.pos()
    
    def mouseMoveEvent(self, event):
        """鼠标移动事件"""
        if self.dragging and self.last_pos:
            dx = event.pos().x() - self.last_pos.x()
            dy = event.pos().y() - self.last_pos.y()
            self.pan_offset[0] += dx
            self.pan_offset[1] += dy
            self.last_pos = event.pos()
            
            # 更新显示
            if self.original_image is not None:
                self.display_image(self.original_image, self.detections, self.fps)
    
    def mouseReleaseEvent(self, event):
        """鼠标释放事件"""
        if event.button() == Qt.LeftButton:
            self.dragging = False
            self.last_pos = None
    
    def contextMenuEvent(self, event):
        """右键菜单事件"""
        if self.original_image is not None:
            self.reset_view()
    
    def reset_view(self):
        """重置视图"""
        self.scale_factor = 1.0
        self.pan_offset = [0, 0]
        
        # 更新显示
        if self.original_image is not None:
            self.display_image(self.original_image, self.detections, self.fps)

class DetectionGUI(QMainWindow):
    def __init__(self, detector):
        super().__init__()
        self.detector = detector
        self.setWindowTitle("YOLOv8 Object Detection with ROS2")
        self.setGeometry(100, 100, 1200, 800)
        
        # 中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QHBoxLayout(central_widget)
        
        # 创建分割器
        splitter = QSplitter(Qt.Horizontal)
        
        # 图像显示区域
        self.image_viewer = DetectionViewer()
        splitter.addWidget(self.image_viewer)
        
        # 检测结果面板
        result_panel = QWidget()
        result_layout = QVBoxLayout(result_panel)
        result_layout.setContentsMargins(5, 5, 5, 5)
        
        # 检测结果标题
        result_title = QLabel("Detection Results")
        result_title.setFont(QFont("Arial", 12, QFont.Bold))
        result_title.setAlignment(Qt.AlignCenter)
        result_layout.addWidget(result_title)
        
        # 检测结果列表
        self.result_list = QListWidget()
        self.result_list.setMinimumWidth(300)
        result_layout.addWidget(self.result_list)
        
        # 控制按钮
        button_layout = QHBoxLayout()
        
        self.save_button = QPushButton("Save Image")
        self.save_button.clicked.connect(self.save_image)
        button_layout.addWidget(self.save_button)
        
        self.reset_button = QPushButton("Reset View")
        self.reset_button.clicked.connect(self.image_viewer.reset_view)
        button_layout.addWidget(self.reset_button)
        
        result_layout.addLayout(button_layout)
        
        splitter.addWidget(result_panel)
        splitter.setSizes([800, 300])
        
        main_layout.addWidget(splitter)
        
        # 状态栏
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready")
        
        # ROS2初始化
        rclpy.init()
        
        # 创建图像处理器节点
        self.image_processor = ImageProcessor(detector, self.update_gui)
        
        # 创建定时器用于处理ROS2事件
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.process_ros_events)
        self.ros_timer.start(10)  # 每10ms处理一次ROS事件
        
        # 显示信息
        self.status_bar.showMessage("GUI initialized. Waiting for images...")
    
    def update_gui(self, image, detections, fps):
        """更新GUI显示"""
        # 更新图像
        self.image_viewer.display_image(image, detections, fps)
        
        # 更新检测结果列表
        self.result_list.clear()
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            label = detection['label']
            conf = detection['confidence']
            item_text = f"{label}: {conf:.2f} @ [{x1}, {y1}, {x2}, {y2}]"
            self.result_list.addItem(item_text)
        
        # 更新状态栏
        self.status_bar.showMessage(f"Detected {len(detections)} objects | FPS: {fps:.1f}")
    
    def process_ros_events(self):
        """处理ROS2事件"""
        rclpy.spin_once(self.image_processor, timeout_sec=0.001)
    
    def save_image(self):
        """保存当前图像"""
        if self.image_viewer.original_image is None:
            QMessageBox.warning(self, "Save Image", "No image available to save.")
            return
        
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Image", "", "Images (*.png *.jpg *.bmp)"
        )
        
        if file_path:
            try:
                # 保存原始图像（带检测结果）
                cv2.imwrite(file_path, self.image_viewer.original_image)
                self.status_bar.showMessage(f"Image saved to {file_path}")
            except Exception as e:
                QMessageBox.critical(self, "Save Error", f"Failed to save image: {str(e)}")
    
    def closeEvent(self, event):
        """关闭事件处理"""
        # 停止ROS2节点
        self.ros_timer.stop()
        self.image_processor.destroy_node()
        rclpy.shutdown()
        event.accept()

def main(args=None):
    # 创建检测器
    detector = YOLODetector()
    
    # 创建Qt应用
    app = QApplication(sys.argv)
    
    # 创建主窗口
    gui = DetectionGUI(detector)
    gui.show()
    
    # 运行应用
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()