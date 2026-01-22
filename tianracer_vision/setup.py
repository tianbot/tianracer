from setuptools import setup

package_name = 'tianracer_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TianRacer Team',
    maintainer_email='you@example.com',
    description='Vision nodes for the TianRacer platform',
    license='MIT',
    extras_require={ 'test': ['pytest', 'flake8'], },
    entry_points={
        'console_scripts': [
            'line_follower = tianracer_vision.line_follower:main'
        ],
    },
)
