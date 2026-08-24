from setuptools import find_packages, setup

package_name = 'packing_common'

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
    maintainer='정정인',
    maintainer_email='jungji0913@khu.ac.kr',
    description='단위/좌표 변환과 config 로더 — 트랙 A~D 공용 기반',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],   # 라이브러리 전용 패키지 (실행 노드 없음)
    },
)
