from setuptools import setup

package_name = 'keyboard_input'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Author Name',
    author_email='author@example.com',
    description='Keyboard input package for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_publisher = keyboard_input.keyboard_publisher:main',
            'keyboard_subscriber = keyboard_input.keyboard_subscriber:main',
        ],
    },
)
