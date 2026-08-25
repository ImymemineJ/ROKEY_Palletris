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
    description='로봇 제어 — cycle · fk · gripper (트랙 B)',
    license='Apache-2.0',
    tests_require=['pytest'],
    # ★ 관제탑은 palletris_orchestrator 하나다 (CLAUDE.md §8).
    #   여기 있던 'orchestrator' 엔트리포인트는 2026-08-26 폐기했다 —
    #   소켓(8765) 시대의 껍데기였고, ROS 2 판이 그 자리를 가져갔다.
    #   근거: docs/2026-08-26_관제탑_이중화_해소.md
    entry_points={'console_scripts': []},
)
