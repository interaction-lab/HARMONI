# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
import os

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['harmoni_tts'],
    package_dir={
        '': 'src',
        'harmoni_tts': os.path.join('src', 'harmoni_tts')
    },
)

setup(**setup_args)
