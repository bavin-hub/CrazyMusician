from setuptools import find_packages, setup

package_name = 'acsi_music'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/adrc_deploy.yaml']),
        ('share/' + package_name + '/config', ['config/adrc_vel_deploy.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bavin',
    maintainer_email='bavinsaravanan@gmail.com',
    description='PX4 ROS 2 acceleration offboard control using cascaded ADRC',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'adrc_offboard = acsi_music.adrc_offboard:main',
            'adrc_deploy_auto = acsi_music.adrc_deploy_auto:main',
            'adrc_deploy_user = acsi_music.adrc_deploy_user:main',
            'adrc_deploy_auto.py = acsi_music.adrc_deploy_auto:main',
            'adrc_deploy_user.py = acsi_music.adrc_deploy_user:main',
            'adrc_vel_deploy_auto = acsi_music.adrc_vel_deploy_auto:main',
            'adrc_vel_deploy_user = acsi_music.adrc_vel_deploy_user:main',
            'adrc_vel_deploy_auto.py = acsi_music.adrc_vel_deploy_auto:main',
            'adrc_vel_deploy_user.py = acsi_music.adrc_vel_deploy_user:main',
        ],
    },
)
