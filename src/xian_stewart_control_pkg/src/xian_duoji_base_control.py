#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
from typing import Dict
import time
from std_msgs.msg import Float32MultiArray

import socket

class ServoController:
    def __init__(self):
        rospy.init_node('xian_duoji_base_control', anonymous=True)
        
        # ROS参数
        # self.port = "/dev/ttyS1"
        # self.baudrate = 9600
        # self.timeout = 3
        #TCP参数
        self.tcp_ip   = '192.168.1.11'   # NE2-T1 静态 IP
        self.tcp_port = 8886             # NE2-T1 本地端口
        self.sock     = None
        self.init_tcp()
        
        # 初始化串口
        # self.serial_port = None
        # self.init_serial()
        # self.init_tcp()
        
        # 舵机控制命令数据结构
        self.init_servo_commands()
        
        # 心跳计数器
        self.counter = 0

        self.xian_arm1_cmd = 0
        self.xian_arm2_cmd = 0
        self.xian_arm3_cmd = 0
        self.xian_arm4_cmd = 0
        self.xian_arm5_cmd = 0
        self.xian_arm6_cmd = 0

        # 请求电压数据结构
        self.CMD_REQ_BATTERY_VOLTAGE = {"frame_head_1": 0x55, # 帧头1，0x55固定不变
                                        "frame_head_2": 0x55, # 帧头2，0x55固定不变
                                        "message_length": 0x02, # 数据长度
                                        "mode": 0x0F,   #指令名：请求控制板电压指令
                                        }

        # 获取电压数据结构
        self.CMD_GET_BATTERY_VOLTAGE = {"frame_head_1": 0x55, # 帧头1，0x55固定不变
                                        "frame_head_2": 0x55, # 帧头2，0x55固定不变
                                        "message_length": 0x04, # 数据长度
                                        "mode": 0x0F,   #：请求控制板电压指令
                                        "param_vol_low": 0x0F,
                                        "param_vol_high": 0x0F,
                                        }
        
        # 定时器
        rospy.Timer(rospy.Duration(1), self.heartbeat_callback)
        rospy.Timer(rospy.Duration(0.1), self.servo_angles_move_callback)
        
        rospy.loginfo("Servo controller node initialized")
        rospy.spin()

    # def init_serial(self):
    #     """初始化串口连接"""
    #     try:
    #         self.serial_port = serial.Serial(
    #             port=self.port,
    #             baudrate=self.baudrate,
    #             bytesize=serial.EIGHTBITS,
    #             parity=serial.PARITY_NONE,
    #             stopbits=serial.STOPBITS_ONE,
    #             timeout=self.timeout
    #         )
    #         rospy.loginfo(f"Successfully connected to serial port {self.port}")
    #     except Exception as e:
    #         rospy.logerr(f"Failed to initialize serial port: {str(e)}")
    #         rospy.signal_shutdown("Serial port initialization failed")

    def init_tcp(self):
        #初始化TCP
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(1)
            self.sock.connect((self.tcp_ip, self.tcp_port))
            rospy.loginfo(f"TCP connected to {self.tcp_ip}:{self.tcp_port}")
        except Exception as e:
            rospy.logerr(f"TCP connect failed: {e}")
            rospy.signal_shutdown("TCP init failed")


    def init_servo_commands(self):
        """初始化舵机控制命令数据结构"""
        # 六个舵机转动数据结构
        self.HEXA_CMD_SERVO_MOVE = {
            "frame_head_1": 0x55, "frame_head_2": 0x55,
            "message_length": 0x17, "mode": 0x03,
            "param_number_hts": 0x06, 
            "param_time_low": 0xE8, "param_time_high": 0x03,
            "param_id_num1": 0x01, "param_plus_low_num1": 0x20, "param_plus_high_num1": 0x03,
            "param_id_num2": 0x02, "param_plus_low_num2": 0x20, "param_plus_high_num2": 0x03,
            "param_id_num3": 0x03, "param_plus_low_num3": 0x20, "param_plus_high_num3": 0x03,
            "param_id_num4": 0x04, "param_plus_low_num4": 0x20, "param_plus_high_num4": 0x03,
            "param_id_num5": 0x05, "param_plus_low_num5": 0x20, "param_plus_high_num5": 0x03,
            "param_id_num6": 0x06, "param_plus_low_num6": 0x20, "param_plus_high_num6": 0x03
        }

    # 编码“请求电压数据结构”
    def encode_CMD_REQ_BATTERY_VOLTAGE(self, data_dict):
        """将字典转换为字节流，按固定字段顺序处理"""
        field_order = [
            "frame_head_1", "frame_head_2", "message_length", "mode"]
        return bytes([data_dict[field] for field in field_order])
    
    # 解码电压
    def decode_CMD_GET_BATTERY_VOLTAGE(self, raw_data: bytes):
        """解析电压响应数据包"""
        if len(raw_data) < 6:  # 最小长度检查（帧头2 + 长度1 + 模式1 + 数据2）
            return None
        
        if raw_data[0] != 0x55 or raw_data[1] != 0x55:
            return None
        
        response = self.CMD_GET_BATTERY_VOLTAGE.copy()
        response.update({
            "message_length": raw_data[2],
            "mode": raw_data[3],
            "param_vol_low": raw_data[4],
            "param_vol_high": raw_data[5]
        })
        return response

    def encode_HEXA_CMD_SERVO_MOVE(self, data_dict: Dict[str, int]) -> bytes:
        """将字典转换为字节流，按固定字段顺序处理"""
        field_order = [
            "frame_head_1", "frame_head_2", "message_length", "mode",
            "param_number_hts", "param_time_low", "param_time_high",
            "param_id_num1", "param_plus_low_num1", "param_plus_high_num1",
            "param_id_num2", "param_plus_low_num2", "param_plus_high_num2",
            "param_id_num3", "param_plus_low_num3", "param_plus_high_num3",
            "param_id_num4", "param_plus_low_num4", "param_plus_high_num4",
            "param_id_num5", "param_plus_low_num5", "param_plus_high_num5",
            "param_id_num6", "param_plus_low_num6", "param_plus_high_num6"
        ]
        return bytes([data_dict[field] for field in field_order])

    def int_to_hex_bytes(self, num):
        """将整数转换为2字节的有符号字节数组（大端序）"""
        bytes_data = num.to_bytes(2, byteorder='big', signed=True)
        return bytes_data[0], bytes_data[1]

    def angle_to_pulse(self, angle):
        """
        将角度(-120°到120°)转换为脉冲值(0-1000)
        其中0-1000脉冲对应0-240°转角
        """
        if angle > 120:
            angle = 120
        elif angle < -120:
            angle = -120

        mapped_angle = angle + 120  # 将范围转换为0-240°
        pulse = int((mapped_angle / 240) * 1000)
        pulse = max(0, min(1000, pulse))
        return self.int_to_hex_bytes(pulse)

    def servo_angles_move_callback(self, event):
        self.xian_arm1_cmd = rospy.get_param("/xian_stewart_control_params_server/xian_arm1_cmd")
        self.xian_arm2_cmd = rospy.get_param("/xian_stewart_control_params_server/xian_arm2_cmd")
        self.xian_arm3_cmd = rospy.get_param("/xian_stewart_control_params_server/xian_arm3_cmd")
        self.xian_arm4_cmd = rospy.get_param("/xian_stewart_control_params_server/xian_arm4_cmd")
        self.xian_arm5_cmd = rospy.get_param("/xian_stewart_control_params_server/xian_arm5_cmd")
        self.xian_arm6_cmd = rospy.get_param("/xian_stewart_control_params_server/xian_arm6_cmd")
            
        try:
            # 设置响应时间
            param_time_high, param_time_low = self.int_to_hex_bytes(100)
            self.HEXA_CMD_SERVO_MOVE["param_time_low"] = param_time_low
            self.HEXA_CMD_SERVO_MOVE["param_time_high"] = param_time_high

            # 设置舵机1-6的旋转角度
            param_plus_high_num1, param_plus_low_num1 =self.angle_to_pulse(self.xian_arm1_cmd)
            param_plus_high_num2, param_plus_low_num2 =self.angle_to_pulse(self.xian_arm2_cmd * -1.0)
            param_plus_high_num3, param_plus_low_num3 =self.angle_to_pulse(self.xian_arm3_cmd)
            param_plus_high_num4, param_plus_low_num4 =self.angle_to_pulse(self.xian_arm4_cmd * -1.0)
            param_plus_high_num5, param_plus_low_num5 =self.angle_to_pulse(self.xian_arm5_cmd)
            param_plus_high_num6, param_plus_low_num6 =self.angle_to_pulse(self.xian_arm6_cmd * -1.0)
                    
            self.HEXA_CMD_SERVO_MOVE["param_plus_low_num1"] = param_plus_low_num1
            self.HEXA_CMD_SERVO_MOVE["param_plus_low_num2"] = param_plus_low_num2
            self.HEXA_CMD_SERVO_MOVE["param_plus_low_num3"] = param_plus_low_num3
            self.HEXA_CMD_SERVO_MOVE["param_plus_low_num4"] = param_plus_low_num4
            self.HEXA_CMD_SERVO_MOVE["param_plus_low_num5"] = param_plus_low_num5
            self.HEXA_CMD_SERVO_MOVE["param_plus_low_num6"] = param_plus_low_num6
            self.HEXA_CMD_SERVO_MOVE["param_plus_high_num1"] = param_plus_high_num1
            self.HEXA_CMD_SERVO_MOVE["param_plus_high_num2"] = param_plus_high_num2
            self.HEXA_CMD_SERVO_MOVE["param_plus_high_num3"] = param_plus_high_num3
            self.HEXA_CMD_SERVO_MOVE["param_plus_high_num4"] = param_plus_high_num4
            self.HEXA_CMD_SERVO_MOVE["param_plus_high_num5"] = param_plus_high_num5
            self.HEXA_CMD_SERVO_MOVE["param_plus_high_num6"] = param_plus_high_num6

            # 发送命令
            tx_data = self.encode_HEXA_CMD_SERVO_MOVE(self.HEXA_CMD_SERVO_MOVE)
            if self.sock:
                # self.serial_port.write(tx_data)
                self.sock.sendall(tx_data)
                print("angle 1: % 0.3f, angle 2: % 0.3f, angle 3: % 0.3f, angle 4: % 0.3f, angle 5: % 0.3f, angle 6: % 0.3f" % 
                              (self.xian_arm1_cmd,self.xian_arm2_cmd,self.xian_arm3_cmd,self.xian_arm4_cmd,self.xian_arm5_cmd,self.xian_arm6_cmd))
            else:
                rospy.logerr("Serial port not available")
                
        except Exception as e:
            rospy.logerr(f"Error processing servo command: {str(e)}")

    def heartbeat_callback(self, event):
        """心跳回调函数，定期更新参数服务器中的心跳值"""
        self.counter = (self.counter + 1) % 1000
        rospy.set_param("/xian_stewart_control_params_server/xian_servo_controller_heart_beat", self.counter)
        print(f"Servo controller heartbeat: {self.counter}")

        ttx_data = self.encode_CMD_REQ_BATTERY_VOLTAGE(self.CMD_REQ_BATTERY_VOLTAGE)
        # self.serial_port.write(ttx_data)
        self.sock.sendall(ttx_data)
        # # 接收响应（带超时重试机制）
        start_time = time.time()
        while time.time() - start_time < 5:  # 5秒超时
            rx_data = self.sock.recv(6)
            if len(rx_data) ==6:  # 预期最小数据长度
                voltage_data = self.decode_CMD_GET_BATTERY_VOLTAGE(rx_data)
                
                if voltage_data:
                    # 计算实际电压值（假设高8位+低8位组成16位整数，单位mV）
                    voltage_mv = (voltage_data["param_vol_high"] << 8) + voltage_data["param_vol_low"]
                    print(f"[RX] Decoded: {voltage_data}")
                    print(f"Battery Voltage: {voltage_mv / 1000:.2f}V")
                    rospy.set_param("/xian_stewart_control_params_server/xian_is_duoji_driver_power", 1)
                    break
                else:
                    print(f"[RX] Invalid data: {rx_data.hex(' ').upper()}")
            # time.sleep(0.01)
        else:
            rospy.set_param("/xian_stewart_control_params_server/xian_is_duoji_driver_power", 0)
            print("Timeout: No valid response received")

    def __del__(self):
        """析构函数，关闭网口"""
        if self.sock:
            self.sock.close()
            rospy.loginfo("TCP socket closed")

if __name__ == '__main__':
    try:
        controller = ServoController()
    except rospy.ROSInterruptException:
        pass
