from setuptools import find_packages, setup

package_name = 'packing_db'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ★ schema.sql 을 share 로 설치한다. 패키지 안에만 두면 --symlink-install 이
        #   아닌 설치본에서 못 찾는다.
        ('share/' + package_name, ['packing_db/schema.sql']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='민범진',
    maintainer_email='jungji0913@khu.ac.kr',
    description='palletris.db 유일 writer + 중재자 (트랙 D)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'db_service_node = packing_db.db_service_node:main',
            'gen_master_db = packing_db.gen_master_db:main',
        ],
    },
)
