from setuptools import setup

package_name = 'tianracer_script'

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
    maintainer='kong',
    maintainer_email='chargerKong@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cmd_to_ackermann = tianracer_script.cmd_vel_to_ackermann_drive:main",
            "wall_follwing = tianracer_script.wall_follwing:main",
            "multi_goal = tianracer_script.multi_goals:main"
        ],
    },
)
