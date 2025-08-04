# ai_voice
AI Voice 是在 rdx x5 开发板基于 ROS2 的实时语音识别和对话，集成了 SenseVoice 语音识别模型和 LLaMA 大语言模型。该系统能够实时捕获音频输入，进行语音识别，并通过大语言模型生成智能回复。

SenseVoice安装和使用参考链接： https://github.com/FunAudioLLM/SenseVoice

llame模型参考文章：https://horizonrobotics.feishu.cn/docx/LQU9dYyjcoXJ9hxJdUYc2l4InEf

TTS参考链接：https://developer.d-robotics.cc/rdk_doc/Robot_development/quick_demo/hobot_tts


  SenseVoice节点  ───> LLaMA处理器节点  ───> TTS文本输出话题

主要组件
1.SenseVoice 节点 (SenseVoice.py)

    实时音频录制和语音识别
    使用 WebRTC VAD 进行语音活动检测
    自动设备查找和管理
    设备播放状态监控

2.LLaMA 处理器节点 (llm.py)

    接收语音识别结果
    与 LLaMA 大语言模型交互
    生成智能回复并发布

3.设备监控模块 (device_monitor.py)

    监控音频设备播放状态
    防止录音时的回音干扰

注意：

    记得提前修该代码录音设备id：SenseVoice.py文件 20行，可直接将默认设备改为自己设备id 或者 使用参数指定

        self.declare_parameter('audio_device_id', -1)  # -1表示自动查找ES7210设备
        self.declare_parameter('target_device_name', 'ES7210')  # 目标设备名称

    提前下载好模型并修改代码里的模型文件路径：llm.py 文件53行 

        '-m', '/root/qwen2.5-1.5b-instruct-q4_k_m.gguf',

启动命令

1：ros2 run ai_voice sense_voice_node --ros-args --log-level debug 

#启动语音识别，通过参数修改日志级别查看详细信息 

2：ros2 run ai_voice llama_processor_node

#启动llm大模型，提前下载好模型并修改代码里的模型文件路径

3：ros2 run hobot_tts hobot_tts --ros-args -p playback_device:="hw:1,0"

#启动文本转语音，使用时注意对应播放设备id