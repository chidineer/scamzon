from setuptools import setup

package_name = 'sml_graph'

setup(
    name=package_name,
    version='0.0.2',  # Updated version
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'matplotlib', 'geometry_msgs', 'nav_msgs', 'std_msgs'],  # Combined install_requires
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@domain.com',
    description='Plot yaw data from multiple topics for Turtlebot3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yaw_listener = sml_graph.yaw_listener:main',  # Ensure the entry point matches your Python file and function
        ],
    },
)
