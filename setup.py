from setuptools import find_packages, setup

package_name = 'urdf_configurator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource', ['resource/RosURDFConfig.ui']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Gahner Holm',
    maintainer_email='daniel.gahner.holm@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'urdf_configurator = urdf_configurator.urdf_configurator:main',
            'urdf_configurator_gui = urdf_configurator.urdf_configurator_gui:main'
        ],
    },
)
