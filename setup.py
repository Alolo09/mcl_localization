from setuptools import find_packages, setup

package_name = 'mcl_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mcl.launch.py']),
        ('share/' + package_name + '/urdf',   ['urdf/mcl_robot.urdf']),
        ('share/' + package_name + '/worlds', ['worlds/mi_mundo.world']),
        ('share/' + package_name + '/maps',   []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alondra',
    maintainer_email='alondra@todo.todo',
    description='MCL Localiaztion',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mcl_node_loca = mcl_localization.mcl_node_loca:main',
            'Map_builder = mcl_localization.Map_builder:main',
            'auto_explorer = mcl_localization.auto_explorer:main',
            'visualizar = mcl_localization.visualizar_particulas:main',
        ],
    },
)
