from setuptools import find_packages, setup

package_name = 'packing_vision'

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
    maintainer='민범진',
    maintainer_email='jungji0913@khu.ac.kr',
    description='인식 노드 — 투입부 등록·검증 + 픽업 좌표 (트랙 D)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = packing_vision.vision_node:main',
        ],
    },
)
