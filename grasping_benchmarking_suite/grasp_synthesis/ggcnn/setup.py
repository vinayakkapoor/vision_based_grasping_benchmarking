from setuptools import setup

package_name = 'ggcnn'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhea',
    maintainer_email='rhea@example.com',
    description='GGCNN implementation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ggcnn_node = ggcnn.ggcnn_node:main',
        ],
    },
)

