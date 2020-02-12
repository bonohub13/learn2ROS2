from setuptools import setup

package_name = 'fibonacci_action_py3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kensuke',
    maintainer_email='ken.saito.0813@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fib_server = fibonacci_action_py3.fib_server:main',
            'fib_client = fibonacci_action_py3.fib_client:main'
            ],
    },
)
