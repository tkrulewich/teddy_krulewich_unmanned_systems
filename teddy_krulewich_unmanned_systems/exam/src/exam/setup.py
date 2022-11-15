from setuptools import setup
from glob import glob
import os

package_name = 'exam'
submodules = 'exam.submodules'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teddy',
    maintainer_email='ekkyv6@mail.umkc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'question2 = exam.question2:main',
            'question3 = exam.question3:main',
            'question4 = exam.question4:main',
            'question5 = exam.question5:main',
            'turtlebot_spawn = exam.turtlebot_spawn:main',
        ],
    },
)
