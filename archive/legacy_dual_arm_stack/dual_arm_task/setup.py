from setuptools import find_packages, setup
import warnings

# 屏蔽系统级 Python 包版本号不规范导致的警告 (如 "1.1build1 is an invalid version")
warnings.filterwarnings("ignore", category=UserWarning, module='setuptools')
warnings.filterwarnings("ignore", message=".*is an invalid version.*")

package_name = 'dual_arm_task'

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
    maintainer='wusiala',
    maintainer_email='wusiala@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
