from setuptools import find_packages, setup

package_name = 'packing_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='정정인 (초안 · 소유: 김병직)',
    maintainer_email='jungji0913@khu.ac.kr',
    description='로봇 제어 / orchestrator (트랙 B)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orchestrator = packing_robot.orchestrator:main',
        ],
    },
)
