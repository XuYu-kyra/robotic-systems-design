import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'color_blob_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student24',
    maintainer_email='kyratsuieu@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'color_blob_detector = color_blob_vision.color_blob_detector:main',
            # 将默认 3D 投影节点切换为“带多帧平滑/确认”的版本
            'blob_depth_to_3d = color_blob_vision.blob_depth_to_3d_smoothed:main',
            # 旧版本仍保留一个可选入口，便于回退/对比
            'blob_depth_to_3d_raw = color_blob_vision.blob_depth_to_3d:main',
            'color_blob_summary = color_blob_vision.color_blob_summary:main',
            'color_blob_markers = color_blob_vision.color_blob_markers:main',
            'color_blob_debug_image = color_blob_vision.color_blob_debug_image:main',
            'color_blob_run_recorder = color_blob_vision.color_blob_run_recorder:main',
            'perception_manager = color_blob_vision.perception_manager:main',
            'task_manager = color_blob_vision.task_manager:main',
        ],
    },
)
