from setuptools import find_packages, setup

package_name = 'my_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','image_detection'],
    zip_safe=True,
    maintainer='stlab',
    maintainer_email='stlab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_first_node = my_controller.my_first_node:main",
            "subcriber= my_controller.subcriber:main",
            "publisher= my_controller.publisher:main",
            "publisher_verif= my_controller.publisher_verif:main",
            "pub_sub = my_controller.pub_sub:main",
            "image_detection = my_controller.image_detection:main"



        ],
    },
)
