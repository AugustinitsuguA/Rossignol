from glob import glob
from setuptools import find_packages, setup

package_name = 'rossignol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahocq',
    maintainer_email='ahocquet@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'points_img = rossignol.pts_arUco:main',
            'detect = rossignol.detect:main',
            'filtre = rossignol_cpp.filtre:main',
            'calibration = rossignol.calibration:main',
            'commande_test = rossignol.commande_test:main',
            'record = rossignol.record:main',
        ],
    },
)
