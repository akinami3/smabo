from setuptools import find_packages, setup

package_name = 'smabo_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='docker',
    maintainer_email='er17026-6673@sti.chubu.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'ノード名（任意） = パッケージ名.pythonファイル名:main'
            'impression_pub = smabo_pkg.impression_pub:main',
            'gyro_sub = smabo_pkg.gyro_sub:main',
            'accel_sub = smabo_pkg.accel_sub:main',
            'magnetic_sub = smabo_pkg.magnetic_sub:main',
            'pupil_move = smabo_pkg.pupil_move:main',
            'pca9685_controller = smabo_pkg.pca9685_controller:main',
            'simple_hand_angle_commander = smabo_pkg.simple_hand_angle_commander:main',
            'cam_stream = smabo_pkg.cam_stream:main',
            'flask_image_display = smabo_pkg.flask_image_display:main',
            'ultrasonic_sensor = smabo_pkg.ultrasonic_sensor:main',
            'head_arrow_key_commander = smabo_pkg.head_arrow_key_commander:main',
        ],
    },
)
