from setuptools import find_packages, setup

package_name = 'ai_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'pyaudio',
        'numpy',
        'webrtcvad',
        'funasr',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sense_voice_node = ai_voice.SenseVoice:main',
            'llama_processor_node = ai_voice.llm:main',
        ],
    },
)
