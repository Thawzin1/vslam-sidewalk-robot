## Standard catkin Python packaging.
##
## This is NOT run directly — `catkin_python_setup()` in CMakeLists.txt invokes
## it. Running `python setup.py install` by hand would install outside the
## workspace and shadow the catkin-managed copy, which is a confusing state to
## debug. Don't.
##
## Its purpose: put `sidewalk_bringup` on the Python path so every other package
## can do
##
##     from sidewalk_bringup.run_logger import RunLogger
##
## and have it resolve both in a source workspace and after installation.

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["sidewalk_bringup"],
    package_dir={"": "src"},
)

setup(**setup_args)
