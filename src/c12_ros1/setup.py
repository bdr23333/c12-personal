from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=["c12_ros1"],
    package_dir={"": "src"},
)

setup(**setup_args)
