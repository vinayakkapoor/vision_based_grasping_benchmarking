from setuptools import setup

package_name = 'mask_based_algo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhea',
    maintainer_email='rhea@example.com',
    description='Python-based mask algorithm with custom messages',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mask_node = mask_based_algo.mask_node:main',
        ],
    },
)

