from setuptools import find_packages, setup

package_name = 'palletris_orchestrator'

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
    maintainer='jin',
    maintainer_email='zldzmfoq100@gmail.com',
    description='Project Palletris 관제탑 — 박스 1개 처리 7단계 상태기계 (1차: B구역)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ros2 run palletris_orchestrator orchestrator_node
            'orchestrator_node = palletris_orchestrator.orchestrator_node:main',
        ],
    },
)
