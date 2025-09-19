from setuptools import setup

package_name = 'uuv_master'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=['toArduino'],  # si no está en un paquete, solo el script
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Antonio',
    maintainer_email='tuemail@example.com',
    description='Nodo Serial en ROS2',
    license='MIT',
    entry_points={
    'console_scripts': [
        'to_arduino = uuv_master.to_arduino:main',
    ],
},

)
