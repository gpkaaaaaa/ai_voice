#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import threading

class AudioDeviceMonitor:
    def __init__(self, device_card=1, device_id=0):
        """
        初始化音频设备监控器
        :param device_card: 声卡编号 (默认1)
        :param device_id: 设备编号 (默认0)
        """
        self.device_card = device_card
        self.device_id = device_id
        self.status_file = f'/proc/asound/card{device_card}/pcm{device_id}p/sub0/status'
        self.last_hw_ptr = 0
        self.last_appl_ptr = 0
        self.last_check_time = 0
        
    def is_device_playing(self):
        """
        检测设备是否正在播放音频
        :return: (is_playing, status_info)
        """
        try:
            if not os.path.exists(self.status_file):
                return False, "设备状态文件不存在"
            
            with open(self.status_file, 'r') as f:
                content = f.read()
                lines = content.strip().split('\n')
                
                # 解析状态信息
                state = "未知"
                owner_pid = "无"
                hw_ptr = 0
                appl_ptr = 0
                
                for line in lines:
                    if line.startswith('state:'):
                        state = line.split(':')[1].strip()
                    elif line.startswith('owner_pid'):
                        owner_pid = line.split(':')[1].strip()
                    elif line.startswith('hw_ptr'):
                        try:
                            hw_ptr = int(line.split(':')[1].strip())
                        except:
                            pass
                    elif line.startswith('appl_ptr'):
                        try:
                            
                            appl_ptr = int(line.split(':')[1].strip())
                        except:
                            pass
                
                # 检查指针是否移动（这表示正在播放）
                current_time = time.time()
                ptr_moved = False
                
                # 只有在上次检查后至少0.1秒才检查指针移动
                if current_time - self.last_check_time > 0.1:
                    ptr_moved = (hw_ptr != self.last_hw_ptr or appl_ptr != self.last_appl_ptr)
                    self.last_hw_ptr = hw_ptr
                    self.last_appl_ptr = appl_ptr
                    self.last_check_time = current_time
                
                # 判断播放状态
                if state == 'RUNNING':
                    return True, f"正在播放(PID:{owner_pid})"
                elif ptr_moved and owner_pid != "0":
                    return True, f"检测到播放(PID:{owner_pid})"
                elif owner_pid != "0":
                    return False, f"设备占用(PID:{owner_pid},状态:{state})"
                else:
                    return False, f"空闲(状态:{state})"
                    
        except Exception as e:
            return False, f"检测失败: {e}"
    
    def is_recording_allowed(self):
        """
        检查是否允许录音（当设备不在播放时允许录音）
        :return: True if recording is allowed, False otherwise
        """
        is_playing, _ = self.is_device_playing()
        return not is_playing
    
    def get_device_status(self):
        """
        获取设备详细状态信息
        :return: 设备状态字符串
        """
        is_playing, status_info = self.is_device_playing()
        if is_playing:
            return f"🔴 设备{self.device_card}-{self.device_id}: 正在播放 ({status_info})"
        else:
            return f"🟢 设备{self.device_card}-{self.device_id}: 空闲 ({status_info})"