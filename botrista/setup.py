from setuptools import find_packages, setup
import glob

package_name = 'botrista'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.*')),
        ('share/' + package_name + '/config', glob.glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cdiorio',
    maintainer_email='carterdiorio2024@u.northwestern.edu',
    description='Main package that makes coffee',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_localizer = botrista.camera_localizer:camera_localizer_entry',
            'pouring = botrista.pouring:main',
            'kettle = botrista.kettle:kettle_entry',
            'cup_detection = botrista.cup_detection:cup_detection_entry',
            'delay_node = botrista.delay_node:delay_entry',
            'handle_detector = botrista.handle_detector:handle_detector_entry',
            'pick_filter = botrista.pick_filter:main',
            'grasp_node = botrista.grasp_node:main',
            'pot_node = botrista.pot:pot_entry',
            'coffee_ground_node = botrista.coffee_grounds:coffee_grounds_entry',
            'run_botrista = botrista.run_botrista:botrista_entry',
        ],
    },
)
