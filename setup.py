from setuptools import find_packages, setup

package_name = 'impulse_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vi',
    maintainer_email='kuligin.viktor.alexandrovich@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ex_hello_ros_node = impulse_pkg.ex_hello_ros:main",

            "ex_publisher_node = impulse_pkg.ex_publisher:main",
            "ex_subscriber_node = impulse_pkg.ex_subscriber:main",

            "ex_turtle_pub_node = impulse_pkg.ex_turtle_pub:main",
            "ex_turtle_sub_node = impulse_pkg.ex_turtle_sub:main",
            "ex_turtle_feedback_node = impulse_pkg.ex_turtle_feedback:main",

            "rviz_point_node = impulse_pkg.rviz_1_point:main",
        ],
    },
)
