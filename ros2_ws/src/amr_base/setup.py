from setuptools import find_packages, setup

package_name = 'amr_base'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/base.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eslam Habashy',
    maintainer_email='eslam22habashy@gmail.com',
    description='Serial driver for the AMR motor controller',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'base_driver = amr_base.base_driver:main',
        ],
    },
)
