import rclpy
import pyaudio
import threading
import time
from queue import Queue
import webrtcvad
import os
import numpy as np
import tempfile
from rclpy.node import Node
from std_msgs.msg import String
from funasr import AutoModel
from .device_monitor import AudioDeviceMonitor

class SenseVoiceNode(Node):
    def __init__(self):
        super().__init__('sense_voice_node')
        
        # 声明参数
        self.declare_parameter('audio_device_id', -1)  # -1表示自动查找ES7210设备
        self.declare_parameter('target_device_name', 'ES7210')  # 目标设备名称
        
        # 音频参数设置
        self.AUDIO_RATE = 16000        # 音频采样率
        self.AUDIO_CHANNELS = 2        # 双声道（立体声）
        self.CHUNK = 1024              # 音频块大小
        self.VAD_MODE = 3              # VAD 模式 (0-3, 数字越大越敏感)
        self.NO_SPEECH_THRESHOLD = 1   # 缩短无效语音阈值到0.8秒，提高响应速度
        self.MIN_SPEECH_LENGTH = 0.5   # 最小语音长度0.5秒，过滤过短语音
        
        # 获取音频设备参数
        self.audio_device_id = self.get_parameter('audio_device_id').get_parameter_value().integer_value
        self.target_device_name = self.get_parameter('target_device_name').get_parameter_value().string_value
        
        # 如果设备ID为-1，自动查找目标设备
        if self.audio_device_id == -1:
            self.audio_device_id = self.find_target_device()
        
        # 全局变量
        self.last_active_time = time.time()
        self.recording_active = True
        self.audio_segments = []       # 存储音频段
        self.is_processing = False     # 防止重复处理
        
        # 设备播放状态监控（监控设备1-0）
        self.device_monitor = AudioDeviceMonitor(device_card=1, device_id=0)
        
        # 初始化 WebRTC VAD
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(self.VAD_MODE)
        
        # 初始化语音识别模型
        self.init_sencevoice_model()
        
        # 显示可用音频设备信息
        self.list_audio_devices()
        
        # 显示当前选择的设备
        if self.audio_device_id == -1:
            self.get_logger().info('使用默认音频设备')
        else:
            self.get_logger().info(f'使用指定音频设备ID: {self.audio_device_id}')
        
        # ROS2 发布者 - 发布识别出的文本
        self.text_publisher = self.create_publisher(String, 'recognized_text', 10)
        
        # 不再需要播放控制订阅者，直接监控设备状态
        
        # 启动音频录制线程
        self.start_audio_recording()
        
        self.get_logger().info('SenseVoice实时语音识别节点已启动')

    def init_sencevoice_model(self):
        """初始化SenseVoice语音识别模型"""
        try:
            model_dir = "/root/.cache/modelscope/hub/models/iic/SenseVoiceSmall"
            self.model_senceVoice = AutoModel(model=model_dir, trust_remote_code=True)
            self.get_logger().info('SenseVoice模型加载成功')
        except Exception as e:
            self.get_logger().error(f'SenseVoice模型加载失败: {e}')
            self.model_senceVoice = None

    def find_target_device(self):
        """自动查找目标音频设备"""
        p = pyaudio.PyAudio()
        try:
            for i in range(p.get_device_count()):
                device_info = p.get_device_info_by_index(i)
                if device_info['maxInputChannels'] > 0:  # 只检查输入设备
                    device_name = device_info['name']
                    if self.target_device_name in device_name:
                        self.get_logger().info(f"自动找到目标设备: ID {i} - {device_name}")
                        return i
            
            # 如果没找到目标设备，使用默认设备
            default_device = p.get_default_input_device_info()
            self.get_logger().warn(f"未找到包含 '{self.target_device_name}' 的设备，使用默认设备: {default_device['name']}")
            return default_device['index']
            
        except Exception as e:
            self.get_logger().error(f"查找音频设备失败: {e}")
            return 0  # 返回设备0作为备选
        finally:
            p.terminate()

    def list_audio_devices(self):
        """列出所有可用的音频设备"""
        p = pyaudio.PyAudio()
        self.get_logger().info("=== 可用音频设备 ===")
        
        for i in range(p.get_device_count()):
            device_info = p.get_device_info_by_index(i)
            if device_info['maxInputChannels'] > 0:  # 只显示输入设备
                self.get_logger().info(f"设备 {i}: {device_info['name']}")
                self.get_logger().info(f"  - 最大输入声道: {device_info['maxInputChannels']}")
                self.get_logger().info(f"  - 默认采样率: {device_info['defaultSampleRate']}")
                self.get_logger().info("---")
        
        # 显示默认设备
        default_device = p.get_default_input_device_info()
        self.get_logger().info(f"默认输入设备: {default_device['name']} (设备ID: {default_device['index']})")
        p.terminate()

    def start_audio_recording(self):
        """启动音频录制线程"""
        self.audio_thread = threading.Thread(target=self.audio_recorder)
        self.audio_thread.daemon = True
        self.audio_thread.start()

    def audio_recorder(self):
        """音频录制线程"""
        try:
            p = pyaudio.PyAudio()
            
            # 根据参数选择音频设备
            try:
                if self.audio_device_id == -1:
                    # 使用默认设备
                    stream = p.open(format=pyaudio.paInt16,
                                  channels=self.AUDIO_CHANNELS,
                                  rate=self.AUDIO_RATE,
                                  input=True,
                                  frames_per_buffer=self.CHUNK)
                    self.get_logger().info("已打开默认音频设备")
                else:
                    # 使用指定设备
                    device_info = p.get_device_info_by_index(self.audio_device_id)
                    self.get_logger().info(f"正在打开设备: {device_info['name']}")
                    
                    stream = p.open(format=pyaudio.paInt16,
                                  channels=self.AUDIO_CHANNELS,
                                  rate=self.AUDIO_RATE,
                                  input=True,
                                  input_device_index=self.audio_device_id,
                                  frames_per_buffer=self.CHUNK)
                    self.get_logger().info(f"已打开指定音频设备: {device_info['name']}")
            except Exception as device_error:
                self.get_logger().error(f"无法打开音频设备 {self.audio_device_id}: {device_error}")
                self.get_logger().info("尝试使用默认设备...")
                stream = p.open(format=pyaudio.paInt16,
                              channels=self.AUDIO_CHANNELS,
                              rate=self.AUDIO_RATE,
                              input=True,
                              frames_per_buffer=self.CHUNK)
            
            audio_buffer = []
            self.get_logger().info("音频录制已开始")
            
            while self.recording_active and rclpy.ok():
                # 首先读取音频数据（避免缓冲区积压）
                data = stream.read(self.CHUNK, exception_on_overflow=False)
                
                # 检查设备是否正在播放（如果播放则丢弃音频数据）
                if not self.device_monitor.is_recording_allowed():
                    is_playing, status_info = self.device_monitor.is_device_playing()
                    if is_playing:
                        self.get_logger().debug(f"设备正在播放，丢弃音频数据: {status_info}")
                    continue  # 直接丢弃这次的音频数据，继续下次循环
                
                audio_buffer.append(data)
                
                # 每 0.3 秒检测一次 VAD（提高响应速度）
                if len(audio_buffer) * self.CHUNK / self.AUDIO_RATE >= 0.3:
                    
                    # 拼接音频数据并检测 VAD
                    raw_audio = b''.join(audio_buffer)
                    vad_result = self.check_vad_activity(raw_audio)
                    
                    if vad_result:
                        self.get_logger().debug("检测到语音活动")
                        self.last_active_time = time.time()
                        self.audio_segments.append(raw_audio)
                    else:
                        self.get_logger().debug("静音中...")
                    
                    audio_buffer = []  # 清空缓冲区
                
                # 检查无效语音时间 - 缩短触发时间，提高实时性
                if (time.time() - self.last_active_time > self.NO_SPEECH_THRESHOLD and 
                    self.audio_segments and not self.is_processing):
                    
                    # 检查语音长度是否足够
                    total_duration = len(self.audio_segments) * 0.3
                    if total_duration >= self.MIN_SPEECH_LENGTH:
                        self.process_audio_realtime()
                    else:
                        self.get_logger().debug(f"语音过短({total_duration:.1f}s)，跳过识别")
                        self.audio_segments.clear()
                    
                    self.last_active_time = time.time()
            
            stream.stop_stream()
            stream.close()
            p.terminate()
            
        except Exception as e:
            self.get_logger().error(f'音频录制错误: {e}')
            # 尝试清理资源
            try:
                if 'stream' in locals():
                    stream.stop_stream()
                    stream.close()
                if 'p' in locals():
                    p.terminate()
            except:
                pass

    def stereo_to_mono(self, stereo_data):
        """将双声道音频转换为单声道"""
        audio_array = np.frombuffer(stereo_data, dtype=np.int16)
        stereo_array = audio_array.reshape(-1, 2)
        mono_array = np.mean(stereo_array, axis=1, dtype=np.int16)
        return mono_array.tobytes()

    def check_vad_activity(self, audio_data):
        """检测 VAD 活动"""
        # 双声道转单声道用于VAD检测
        mono_data = self.stereo_to_mono(audio_data)
        
        # 将音频数据分块检测，设置有效激活率rate=35%（稍微降低敏感度）
        num, rate = 0, 0.5
        step = int(self.AUDIO_RATE * 0.02)  # 20ms 块大小
        flag_rate = round(rate * len(mono_data) // step)

        for i in range(0, len(mono_data), step):
            chunk = mono_data[i:i + step]
            if len(chunk) == step:
                if self.vad.is_speech(chunk, sample_rate=self.AUDIO_RATE):
                    num += 1

        return num > flag_rate

    def process_audio_realtime(self):
        """实时处理音频（无文件保存）"""
        if not self.audio_segments or self.is_processing:
            return
        
        self.is_processing = True
        
        try:
            # 合并所有音频段
            stereo_audio = b''.join(self.audio_segments)
            mono_audio = self.stereo_to_mono(stereo_audio)
            
            # 清空音频缓存，释放内存
            self.audio_segments.clear()
            
            self.get_logger().info(f"开始识别音频段，长度: {len(mono_audio)} 字节")
            
            # 使用临时文件进行识别（识别后自动删除）
            recognition_thread = threading.Thread(
                target=self.speech_recognition_from_memory, 
                args=(mono_audio,)
            )
            recognition_thread.start()
            
        except Exception as e:
            self.get_logger().error(f"音频处理失败: {e}")
            self.is_processing = False

    def speech_recognition_from_memory(self, mono_audio_data):
        """从内存中的音频数据进行语音识别"""
        temp_file = None
        try:
            if self.model_senceVoice is None:
                self.get_logger().error("SenseVoice模型未加载，跳过语音识别")
                return
            
            # 创建临时文件（系统会自动管理）
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                # 写入WAV头和音频数据
                self.write_wav_file(temp_file.name, mono_audio_data)
                temp_filename = temp_file.name
            
            # 语音识别
            res = self.model_senceVoice.generate(
                input=temp_filename,
                cache={},
                language="zn",
                use_itn=False,
            )
            
            # 立即删除临时文件
            os.unlink(temp_filename)
            
            # 提取识别结果文本
            recognized_text = res[0]['text'].split(">")[-1].strip()
            
            if recognized_text:
                self.get_logger().info(f"实时识别结果: {recognized_text}")
                
                # 发布识别结果到ROS话题
                text_msg = String()
                text_msg.data = recognized_text
                self.text_publisher.publish(text_msg)
                
                self.get_logger().info(f"已发布文本: {recognized_text}")
            else:
                self.get_logger().debug("识别结果为空")
            
        except Exception as e:
            self.get_logger().error(f"语音识别失败: {e}")
            # 确保临时文件被删除
            if temp_file and os.path.exists(temp_file.name):
                try:
                    os.unlink(temp_file.name)
                except:
                    pass
        finally:
            self.is_processing = False

    def write_wav_file(self, filename, audio_data):
        """将音频数据写入WAV文件"""
        import wave
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(1)  # 单声道
            wf.setsampwidth(2)  # 16-bit PCM
            wf.setframerate(self.AUDIO_RATE)
            wf.writeframes(audio_data)

    # playback_control_callback 已移除，改为直接监控设备播放状态

    def stop_recording(self):
        """停止录制"""
        self.recording_active = False
        # 内存会在程序退出时自动清理，重点是确保音频设备正确释放
        self.get_logger().info("音频录制已停止，设备资源将自动释放")

def main(args=None):
    rclpy.init(args=args)
    sense_voice_node = None
    
    try:
        sense_voice_node = SenseVoiceNode()
        rclpy.spin(sense_voice_node)
    except KeyboardInterrupt:
        if sense_voice_node:
            sense_voice_node.get_logger().info("接收到停止信号")
    except Exception as e:
        if sense_voice_node:
            sense_voice_node.get_logger().error(f"程序异常: {e}")
        else:
            print(f"初始化失败: {e}")
    finally:
        if sense_voice_node:
            sense_voice_node.stop_recording()
            sense_voice_node.destroy_node()
        # 检查rclpy是否还在运行，避免重复shutdown
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # 忽略shutdown相关的异常

if __name__ == '__main__':
    main()
    