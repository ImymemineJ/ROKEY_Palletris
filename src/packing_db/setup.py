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
    # ★ site-packages 안에도 같이 깐다 (2026-08-26).
    #   find_packages() 는 .py 만 가져가서, 설치본의 패키지 디렉터리에는 schema.sql 이
    #   아예 없었다. share 폴백이 고쳐졌으니 없어도 돌지만, 두 경로 다 살려 두면
    #   ROS 를 source 안 한 상태에서도 모듈 옆에서 바로 찾힌다.
    package_data={package_name: ['schema.sql']},
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
