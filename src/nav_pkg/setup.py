from setuptools import find_packages, setup

package_name = 'nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ddp22',
    maintainer_email='d.damora3@studenti.unisa.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "navigation = nav_pkg.navigation:main",
            "next_waypoint = nav_pkg.next_waypoint:main",
            "qr_code_manager = nav_pkg.qr_code_manager:main",
            "recovery_node = nav_pkg.qr_recovery:main",
            "starter = nav_pkg.starter:main", 
            "checkpoint = nav_pkg.checkpoint:main"
        ],
    },
)
