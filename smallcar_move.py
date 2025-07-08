# 导入ROS2相关模块
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import struct
import math
import numpy as np

# 导入ROS2消息类型
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import time

# 全局常量
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
RECEIVE_DATA_SIZE = 24
ACCEL_RATIO = 16384.0  # ±2g对应16384 LSB/g
GYROSCOPE_RATIO = 0.0010652  # ±500°/s对应65.5 LSB/°/s，转换为rad/s

class MickRobotNode(Node):
    def __init__(self):
        """节点初始化"""
        super().__init__('mickrobot')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('cmdvel_topic', '/cmd_vel'),
                ('pub_odom_topic', '/odom'),
                ('imu_topic', '/imu/data_raw'),
                ('voltage_topic', '/PowerVoltage'),
                ('dev', '/dev/ttyUSB0'),
                ('baud', 115200),
                ('time_out', 1000),
                ('hz', 50),
                ('odom_frame_id', 'odom'),
                ('robot_frame_id', 'base_footprint'),
                ('gyro_frame_id', 'gyro_link')
            ]
        )

        # 获取参数值
        self.sub_cmdvel_topic = self.get_parameter('cmdvel_topic').value
        self.pub_odom_topic = self.get_parameter('pub_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.voltage_topic = self.get_parameter('voltage_topic').value
        self.dev = self.get_parameter('dev').value
        self.baud = self.get_parameter('baud').value
        self.time_out = self.get_parameter('time_out').value
        self.hz = self.get_parameter('hz').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.gyro_frame_id = self.get_parameter('gyro_frame_id').value

        # 初始化变量
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_w = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vw = 0.0
        self.power_voltage = 0.0
        self.buffer = bytearray()
        self.last_time = self.get_clock().now()
        
        # 创建发布者
        self.odom_pub = self.create_publisher(Odometry, self.pub_odom_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.voltage_pub = self.create_publisher(Float32, self.voltage_topic, 10)
        
        # 创建订阅者
        self.cmd_vel_sub = self.create_subscription(
            Twist, self.sub_cmdvel_topic, self.cmd_vel_callback, 10)
        
        # 初始化TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 初始化串口
        try:
            self.serial_port = serial.Serial(
                port=self.dev,
                baudrate=self.baud,
                timeout=self.time_out/1000
            )
            self.get_logger().info(f"串口 {self.serial_port.port} 打开成功")
            self.serial_port.flushInput()
        except serial.SerialException as e:
            self.get_logger().error(f"打开串口失败: {str(e)}")
            raise SystemExit(-1)
        
        # 创建定时器
        self.timer = self.create_timer(1.0/self.hz, self.main_loop)
        self.get_logger().info("MickRobot 节点已启动")

    def cmd_vel_callback(self, msg):
        """
        速度指令回调函数
        :param msg: Twist消息
        """
        # 构建数据包 (11字节)
        send_data = bytearray(11)
        
        # 填充帧头
        send_data[0] = FRAME_HEADER
        send_data[1] = 0  # AutoRecharge (禁用自动回充)
        send_data[2] = 0  # 预留位
        
        # 填充速度数据 (放大1000倍)
        vx_int = int(msg.linear.x * 1000)
        vy_int = int(msg.linear.y * 1000)
        vw_int = int(msg.angular.z * 1000)
        
        # X方向速度
        send_data[3] = (vx_int >> 8) & 0xFF
        send_data[4] = vx_int & 0xFF
        
        # Y方向速度
        send_data[5] = (vy_int >> 8) & 0xFF
        send_data[6] = vy_int & 0xFF
        
        # Z方向角速度
        send_data[7] = (vw_int >> 8) & 0xFF
        send_data[8] = vw_int & 0xFF
        
        # 计算校验和
        send_data[9] = self.calculate_checksum(send_data[0:9])
        
        # 填充帧尾
        send_data[10] = FRAME_TAIL
        
        # 发送数据
        try:
            self.serial_port.write(send_data)
        except Exception as e:
            self.get_logger().error(f"发送速度指令失败: {str(e)}")

    def calculate_checksum(self, data):
        """计算校验和 (异或)"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def parse_received_data(self, data):
        """解析接收到的数据"""
        try:
            # 检查帧头和帧尾
            if data[0] != FRAME_HEADER or data[23] != FRAME_TAIL:
                return False
                
            # 验证校验和
            if data[22] != self.calculate_checksum(data[0:22]):
                return False
                
            # 解析速度数据
            self.vx = self.parse_speed(data[2], data[3])
            self.vy = self.parse_speed(data[4], data[5])
            self.vw = self.parse_speed(data[6], data[7])
            
            # 解析IMU加速度数据
            accel_x = self.parse_imu(data[8], data[9]) / ACCEL_RATIO
            accel_y = self.parse_imu(data[10], data[11]) / ACCEL_RATIO
            accel_z = self.parse_imu(data[12], data[13]) / ACCEL_RATIO
            
            # 解析IMU角速度数据
            gyro_x = self.parse_imu(data[14], data[15]) * GYROSCOPE_RATIO
            gyro_y = self.parse_imu(data[16], data[17]) * GYROSCOPE_RATIO
            gyro_z = self.parse_imu(data[18], data[19]) * GYROSCOPE_RATIO
            
            # 解析电池电压
            voltage_raw = (data[21] << 8) | data[20]
            self.power_voltage = voltage_raw / 1000.0
            
            # 发布IMU数据
            self.publish_imu(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"解析数据失败: {str(e)}")
            return False

    def parse_speed(self, high_byte, low_byte):
        """解析速度值"""
        value = (high_byte << 8) | low_byte
        if value & 0x8000:  # 检查符号位
            value -= 0x10000  # 转换为有符号整数
        return value / 1000.0  # 还原为实际值

    def parse_imu(self, high_byte, low_byte):
        """解析IMU值"""
        value = (high_byte << 8) | low_byte
        if value & 0x8000:  # 检查符号位
            value -= 0x10000  # 转换为有符号整数
        return value

    def publish_odometry(self):
        """发布里程计信息和TF变换"""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        
        # 更新位置
        self.position_x += (self.vx * math.cos(self.position_w) - 
                            self.vy * math.sin(self.position_w)) * dt
        self.position_y += (self.vx * math.sin(self.position_w) + 
                            self.vy * math.cos(self.position_w)) * dt
        self.position_w += self.vw * dt
        
        # 归一化角度
        self.position_w = math.atan2(math.sin(self.position_w), math.cos(self.position_w))
        
        # 创建里程计消息
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.robot_frame_id
        
        # 设置位置
        odom.pose.pose.position.x = self.position_x
        odom.pose.pose.position.y = self.position_y
        odom.pose.pose.position.z = 0.0
        
        # 设置方向 (四元数)
        cy = math.cos(self.position_w * 0.5)
        sy = math.sin(self.position_w * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        
        # 设置速度
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vw
        
        # 发布里程计
        self.odom_pub.publish(odom)
        
        # 发布TF变换
        self.publish_tf_transform(now.to_msg())
        
        # 发布电压
        voltage_msg = Float32()
        voltage_msg.data = self.power_voltage
        self.voltage_pub.publish(voltage_msg)

    def publish_tf_transform(self, timestamp):
        """发布TF变换"""
        t = TransformStamped()
        
        # 设置头信息
        t.header.stamp = timestamp
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.robot_frame_id
        
        # 设置位置
        t.transform.translation.x = self.position_x
        t.transform.translation.y = self.position_y
        t.transform.translation.z = 0.0
        
        # 设置方向 (四元数)
        cy = math.cos(self.position_w * 0.5)
        sy = math.sin(self.position_w * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        
        # 发布TF变换
        self.tf_broadcaster.sendTransform(t)

    def publish_imu(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
        """发布IMU数据"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.gyro_frame_id
        
        # 设置加速度
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        
        # 设置角速度
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        
        # 设置协方差矩阵
        imu_msg.linear_acceleration_covariance[0] = 0.01
        imu_msg.linear_acceleration_covariance[4] = 0.01
        imu_msg.linear_acceleration_covariance[8] = 0.01
        
        imu_msg.angular_velocity_covariance[0] = 0.01
        imu_msg.angular_velocity_covariance[4] = 0.01
        imu_msg.angular_velocity_covariance[8] = 0.01
        
        # 发布IMU
        self.imu_pub.publish(imu_msg)

    def main_loop(self):
        """主循环处理串口数据"""
        # 读取串口数据
        if self.serial_port.in_waiting > 0:
            try:
                # 读取可用数据
                data = self.serial_port.read(self.serial_port.in_waiting)
                self.buffer.extend(data)
                
                # 查找完整的数据帧 (24字节)
                while len(self.buffer) >= RECEIVE_DATA_SIZE:
                    # 查找帧头位置
                    header_index = -1
                    for i in range(len(self.buffer) - RECEIVE_DATA_SIZE + 1):
                        if self.buffer[i] == FRAME_HEADER:
                            header_index = i
                            break
                    
                    # 如果找不到帧头，清除缓冲区
                    if header_index == -1:
                        self.buffer.clear()
                        break
                    
                    # 检查帧尾
                    if header_index + 23 < len(self.buffer) and self.buffer[header_index + 23] == FRAME_TAIL:
                        # 提取完整帧并解析
                        frame = self.buffer[header_index:header_index + RECEIVE_DATA_SIZE]
                        if self.parse_received_data(frame):
                            self.publish_odometry()
                        
                        # 移除已处理的数据
                        del self.buffer[:header_index + RECEIVE_DATA_SIZE]
                    else:
                        # 保留帧头，移除前面的数据
                        del self.buffer[:header_index]
                        break
            
            except Exception as e:
                self.get_logger().error(f"处理串口数据时出错: {str(e)}")
                self.buffer.clear()

def main(args=None):
    rclpy.init(args=args)
    node = MickRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()