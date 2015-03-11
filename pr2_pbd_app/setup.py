from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['pr2_pbd_app'],
        package_dir={ '' : 'src' }
    )

setup(**d)