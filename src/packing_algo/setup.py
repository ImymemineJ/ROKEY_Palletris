from setuptools import find_packages, setup

package_name = 'packing_algo'

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
    description='heightmap 기반 3D 패킹 알고리즘 (트랙 C)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'algo_service_node = packing_algo.algo_service_node:main',
            'rehearsal_check = packing_algo.preflight:main',
            'live_view = packing_algo.live_view:main',
            'robustness = packing_algo.robustness:main',
            'report = packing_algo.report:main',
            'ceiling = packing_algo.ceiling:main',
        ],
    },
)
