from setuptools import setup

# read the contents of your README file
from pathlib import Path
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

setup(
    name='abb_motion_program_exec',
    version='0.3.0',
    description='Simple module to execute motion commands on ABB robots and log results',
    url='https://github.com/johnwason/abb_motion_program_exec',
    py_modules=['abb_motion_program_exec_client'],
    install_requires=[
        'requests',
        'numpy'
    ],
    long_description=long_description,
    long_description_content_type='text/markdown'
)