# from setuptools import setup

# package_name = 'miabot_node'

# setup(
#     name=package_name,
#     version='0.0.1',
#     packages=[],
#     py_modules=[
#         'miabot_node',
#     ],
#     install_requires=[
#         'setuptools',
#     ],
#     entry_points={
#         'console_scripts': [
#             'miabot_node = miabot_node:main',
#         ],
#     },
# )

from setuptools import find_packages, setup

package_name = 'miabot_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='szymon',
    maintainer_email='szymon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
