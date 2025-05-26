import os                                                # by launch
from glob import glob                                    # by launch

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
        (os.path.join('share', package_name), glob('launch/*.launch.py')), # by launch
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
            "rviz_pose_node = impulse_pkg.rviz_2_pose:main",
            "rviz_geom_node = impulse_pkg.rviz_3_marker_geom:main",
            "rviz_text_node = impulse_pkg.rviz_4_marker_text:main",
            "rviz_car_2_sim = impulse_pkg.rviz_5_car_sim_2:main",

            "con_1_convert_node = impulse_pkg.control_1a_converter:main",
            "con_1_rviz_node = impulse_pkg.control_1b_rviz:main",

            "num_keyboard_node = impulse_pkg.num_keyboard:main",
            "con_3_main_node = impulse_pkg.control_3_main:main",
            "con_3_object_node = impulse_pkg.control_3_object:main",
            "con_3_goal_node = impulse_pkg.control_3_goal:main",

            "con_4_main_node = impulse_pkg.control_4_main:main",
            "con_4_hunter_node = impulse_pkg.control_4_hunter:main",
            "con_4_hunter2_node = impulse_pkg.control_4_hunter_2:main",
            "con_4_goal_node = impulse_pkg.control_4_goal:main",

            "alg_a_node = impulse_pkg.alg_1:main",

        ],
    },
)
