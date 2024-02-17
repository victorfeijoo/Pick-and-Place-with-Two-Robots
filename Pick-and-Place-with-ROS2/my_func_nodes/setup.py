from setuptools import setup

package_name = 'my_func_nodes'

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
    maintainer='mario',
    maintainer_email='mario.sgarcia@alumnos.upm.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "control_robot_exec = my_func_nodes.control_robot_master:main",
        "camera_exec = my_func_nodes.camera:main",
	"camera_detection = my_func_nodes.camera_pub_pos:main",
	"interfaz_exec = my_func_nodes.interfaz_menu:main"
        ],
    },
)
