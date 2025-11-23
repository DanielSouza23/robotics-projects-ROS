from setuptools import setup
from setuptools import setup, find_packages


package_name = 'custom_joint_control'

setup(
    name=package_name,
    version='1.4.0',
    packages=find_packages(),  
    include_package_data=True,
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['custom_joint_control/launch/eff_control.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'service_msgs',
        'trimesh',      
        'moveit_msgs',  
    ],    
    zip_safe=True,
    maintainer='Daniel Souza',
    maintainer_email='r.souzadaniel@outlook.com',
    description='Joint velocity control system for xArm7 with pose tracking.',
    license='Stevens',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_eff_pose_control = custom_joint_control.main_eff_pose_control:main',
            'force_velocity_adv = custom_joint_control.force_velocity_adv:main',
            'pose_publisher = custom_joint_control.main_eff_pose_control:main',
            'ros2_hybrid_joint_control = custom_joint_control.force_velocity_adv:main'
            'compute_sdf = custom_joint_control.compute_sdf:main',
        ],
    },
)
