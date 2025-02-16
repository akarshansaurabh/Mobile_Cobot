from setuptools import find_packages, setup

package_name = 'detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','numpy','opencv-python','onnxruntime',],
    zip_safe=True,
    maintainer='akarshan',
    maintainer_email='akarshandream5@gmail.com',
    description='Classical Image Processing and YOLOv8 for CPU-Only Real-Time Inference',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov8_node = detection.yolov8_node:main',
            'image_processing_node = detection.image_processing_node:main'
        ],
    },
)
