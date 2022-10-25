from setuptools import setup

package_name = 'homework6'
submodules = 'homework6.submodules'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'question1 = homework6.question1:main',
            'question2 = homework6.question2:main',
            'question3 = homework6.question3:main',
            'question4 = homework6.question4:main'
        ],
    },
)
