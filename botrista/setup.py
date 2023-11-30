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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_localizer = botrista.camera_localizer:camera_localizer_entry',
            'pouring = botrista.pouring:main',
            'kettle = botrista.kettle:kettle_entry',
            'tag_transform_lookup = botrista.tag_transform_lookup:tag_transform_entry',
            'cup_detection = botrista.cup_detection:cup_detection_entry',
            'delay_node = botrista.delay_node:delay_entry',
            'handle_detector = botrista.handle_detector:handle_detector_entry',
        ],
    },
)
