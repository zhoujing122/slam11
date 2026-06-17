from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mujoco_sim'

# 递归收集 models 目录
def collect_model_files(directory='models'):
    model_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            src_path = os.path.join(root, file)
            rel_path = os.path.relpath(src_path, directory)
            dest_dir = os.path.join('share', package_name, 'models', os.path.dirname(rel_path))
            model_files.append((dest_dir, [src_path]))
    return model_files

# 收集所有数据文件
data_files = [
    # ROS 2 package resource
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    # package.xml
    (os.path.join('share', package_name), ['package.xml']),
    # launch 文件
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
] + collect_model_files()  # 添加 models 文件

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@gonfuture.com',
    description='MuJoCo simulation node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_sim = mujoco_sim.mujoco_quadruped_env:main',
            'sim_lidar_publisher = mujoco_sim.sim_lidar_publisher:main',
            'sim_imu_bridge = mujoco_sim.sim_imu_bridge:main',
            'slam_tf_bridge = mujoco_sim.slam_tf_bridge:main',
            'imu_converter = mujoco_sim.imu_converter:main',
            'pointcloud_merger = mujoco_sim.pointcloud_merger:main',
            'fake_lidar_triplet_publisher = mujoco_sim.fake_lidar_triplet_publisher:main',
            'auto_navigator = mujoco_sim.auto_navigator:main',
        ],
    },
)
