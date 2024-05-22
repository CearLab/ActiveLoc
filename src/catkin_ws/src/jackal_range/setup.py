
#!/usr/bin/env python
# license removed for brevity

# imports
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=['range'],
    package_dir={'': 'nodes/modules'},    
)

setup(**setup_args)