import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pacejka_sysid'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'params'),
            glob(os.path.join('params', '*.yaml'))),
        # models/<racecar_version>/*.txt 를 디렉토리 구조 그대로 설치한다.
        # 원본은 catkin devel-space 의 소스 트리를 직접 읽었지만 ament 에서는 불가하므로
        # share 를 템플릿 원본으로 두고 실행 시 output_dir 로 복사해 쓴다 (paths.py 참고).
    ] + [
        (os.path.join('share', package_name, os.path.dirname(f)), [f])
        for f in glob(os.path.join('models', '*', '*.txt'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='jangjunhyeok518@gmail.com',
    description='On-track Pacejka tire parameter identification (ForzaETH On-Track-SysID port)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pacejka_sysid_node = pacejka_sysid.pacejka_sysid_node:main',
        ],
    },
)
