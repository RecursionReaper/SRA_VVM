from setuptools import find_packages, setup

package_name = 'image_recognition'

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
        'opencv-python',   
        'cv_bridge',
    ],
    zip_safe=True,
    maintainer='kartikey',
    maintainer_email='kartikeypathak.imp@gmail.com',
    description='Package for image recognition using ROS 2 and OpenCV',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_subscriber = image_recognition.image_subscriber:main',
            'direct_camera_input = image_recognition.direct_camera_input:main',
            'image_subscriber_2= image_recognition.image_subscriber_2:main'

        ],
    },
)

