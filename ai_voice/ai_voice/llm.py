#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import queue
import time
import os


class LlamaProcessorNode(Node):
    def __init__(self):
        super().__init__('llama_processor_node')
        
        # 创建订阅者 - 适配SenseVoice的输出
        self.subscription = self.create_subscription(
            String,
            'recognized_text',  # 订阅SenseVoice发布的话题
            self.prompt_callback,
            10
        )
        
        # 创建发布者，发布LLaMA的回复
        self.response_publisher = self.create_publisher(
            String,
            'tts_text',
            10
        )
        
        # 不再需要播放控制发布者，语音节点直接监控设备状态
        
        # LLaMA进程相关
        self.llama_process = None
        self.llama_input_queue = queue.Queue()
        self.llama_output_queue = queue.Queue()
        self.is_llama_running = False
        self.response_count = 0  # 计数器，跳过回复
        
        # 启动LLaMA进程
        self._start_llama_process()
        
        # 启动处理线程
        self.processing_thread = threading.Thread(target=self._process_llama_io)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        
        self.get_logger().info('LLaMA处理器节点已启动')
        self.get_logger().info('正在监听 /recognized_text 话题...')

    def _start_llama_process(self):
        """启动LLaMA进程"""
        try:
            # LLaMA命令
            llama_cmd = [
                'llama-cli',
                '-m', '/root/qwen2.5-1.5b-instruct-q4_k_m.gguf',
                '-n', '512',     # 限制每次最多生成100个token
                '-c', '4096',    # 上下文窗口
                '-p', '你是一个陪伴机器人，名字叫小云',  # 系统提示
                '-i',            # 交互式模式，必须有这个参数才能对话
                '-co',           # 对话优化
                '-cnv',          # 对话模式
                '--threads', '6'
            ]
            
            self.get_logger().info('启动LLaMA进程...')
            self.llama_process = subprocess.Popen(
                llama_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            self.is_llama_running = True
            self.get_logger().info('LLaMA进程启动成功')
            
        except Exception as e:
            self.get_logger().error(f'启动LLaMA进程失败: {e}')
            self.is_llama_running = False

    def prompt_callback(self, msg):
        """处理接收到的语音识别结果"""
        if not self.is_llama_running or not self.llama_process:
            self.get_logger().warn('LLaMA进程未运行，无法处理输入')
            return
            
        prompt_text = msg.data.strip()
        
        # 过滤空白或无效输入
        if not prompt_text or len(prompt_text) < 2:
            self.get_logger().debug(f'输入过短，跳过: "{prompt_text}"')
            return
            
        self.get_logger().info(f'收到语音输入: "{prompt_text}"')
        
        # 将输入放入队列
        try:
            self.llama_input_queue.put_nowait(prompt_text)
            self.get_logger().info('已将输入加入LLaMA处理队列')
        except queue.Full:
            self.get_logger().warn('LLaMA输入队列已满，丢弃输入')

    def _process_llama_io(self):
        """处理LLaMA的输入输出"""
        while self.is_llama_running and self.llama_process:
            try:
                # 处理输入
                try:
                    input_text = self.llama_input_queue.get(timeout=1)
                    if input_text:
                        self._send_to_llama(input_text)
                except queue.Empty:
                    pass
                
                # 处理输出
                self._read_llama_output()
                
            except Exception as e:
                self.get_logger().error(f'LLaMA IO处理错误: {e}')
                time.sleep(1)

    def _send_to_llama(self, text):
        """发送文本到LLaMA"""
        try:
            if self.llama_process and self.llama_process.poll() is None:
                # 发送输入到LLaMA
                input_line = f"{text}\n"
                self.llama_process.stdin.write(input_line)
                self.llama_process.stdin.flush()
                self.get_logger().info(f'已发送到LLaMA: {text}')
            else:
                self.get_logger().error('LLaMA进程已停止')
                self.is_llama_running = False
        except Exception as e:
            self.get_logger().error(f'发送到LLaMA失败: {e}')

    def _read_llama_output(self):
        """读取LLaMA的输出"""
        try:
            if self.llama_process and self.llama_process.poll() is None:
                # 非阻塞读取输出
                import select
                if select.select([self.llama_process.stdout], [], [], 0.1)[0]:
                    output_line = self.llama_process.stdout.readline()
                    if output_line and output_line.strip():
                        output_text = output_line.strip()
                        self.get_logger().info(f'LLaMA回复: {output_text}')
                        
                                                # 清理LLaMA输出格式，移除 '> ' 前缀
                        clean_text = output_text
                        if clean_text.startswith('> '):
                            clean_text = clean_text[2:].strip()  # 移除 '> ' 前缀
                        
                        # 过滤无效回复（空内容或只有符号）
                        if clean_text and len(clean_text.strip()) > 0 and clean_text.strip() != '>':
                            self.response_count += 1
                            
                            # 跳过前三条回复（系统初始化回复）
                            if self.response_count <= 4:
                                self.get_logger().info(f'跳过第{self.response_count}条系统回复: "{clean_text}"')
                                
                                # 在跳过第三条后，主动发布自定义问候语
                                if self.response_count == 4:
                                    custom_greeting = "你好！我是小云，一个陪伴机器人。请问有什么可以帮助你的吗？"
                                    response_msg = String()
                                    response_msg.data = custom_greeting
                                    self.response_publisher.publish(response_msg)
                                    self.get_logger().info(f'发布自定义问候语: {custom_greeting}')
                                return
                            
                            # 发布后续所有回复
                            response_msg = String()
                            response_msg.data = clean_text
                            self.response_publisher.publish(response_msg)
                            
                            self.get_logger().info(f'已发布到 /tts_text: {clean_text}')
                        else:
                            self.get_logger().warn(f'跳过无效回复: "{output_text}"')
        except Exception as e:
            self.get_logger().error(f'读取LLaMA输出失败: {e}')



    def _stop_llama_process(self):
        """停止LLaMA进程"""
        if self.llama_process:
            try:
                self.llama_process.terminate()
                self.llama_process.wait(timeout=5)
                self.get_logger().info('LLaMA进程已停止')
            except subprocess.TimeoutExpired:
                self.llama_process.kill()
                self.get_logger().warn('强制终止LLaMA进程')
            except Exception as e:
                self.get_logger().error(f'停止LLaMA进程失败: {e}')
            finally:
                self.llama_process = None
                self.is_llama_running = False

    def on_shutdown(self):
        """节点关闭时的清理"""
        self.get_logger().info('正在关闭LLaMA处理器节点...')
        self.is_llama_running = False
        self._stop_llama_process()

def main(args=None):
    rclpy.init(args=args)
    llama_node = None
    
    try:
        # 创建LLaMA处理器节点
        llama_node = LlamaProcessorNode()
        
        # 运行节点
        rclpy.spin(llama_node)
        
    except KeyboardInterrupt:
        if llama_node:
            llama_node.get_logger().info('接收到停止信号')
    except Exception as e:
        if llama_node:
            llama_node.get_logger().error(f'程序异常: {e}')
        else:
            print(f'初始化失败: {e}')
    finally:
        # 清理资源
        if llama_node:
            llama_node.on_shutdown()
            llama_node.destroy_node()
        
        # 安全shutdown
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
