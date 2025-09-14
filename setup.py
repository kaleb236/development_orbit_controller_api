from setuptools import find_packages, setup

package_name = 'development_orbit_controller_api'
orbit_api = f"{package_name}/orbit_api"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, orbit_api],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ovali',
    maintainer_email='calebndatimana@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blockly_server = development_orbit_controller_api.blockly_server:main'
        ],
    },
)
