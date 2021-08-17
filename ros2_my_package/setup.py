from setuptools import setup

package_name = 'ros2_my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=['scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ito-masaki',
    maintainer_email='ito.masaki@em.ci.ritsumei.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker=scripts.talker:main',
        ],
    },
)
