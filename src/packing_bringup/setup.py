import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'packing_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # ★ system.yaml 은 packing_common.config 가 소스 트리에서 찾는 것이 정본이다.
        #   여기 설치본은 소스 없이 install 만 배포할 때의 대비책이다.
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='팀 공용',
    maintainer_email='jungji0913@khu.ac.kr',
    description='ROS 2 그래프 기동 (launch + system.yaml)',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
