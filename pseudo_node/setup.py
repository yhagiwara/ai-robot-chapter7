from setuptools import setup

package_name = 'pseudo_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
            'manipulation_node = pseudo_node.manipulation_node:main',
            'navigation_node = pseudo_node.navigation_node:main',
            'vision_node = pseudo_node.vision_node:main',
            'voice_node = pseudo_node.voice_node:main'
        ],
    },
)
