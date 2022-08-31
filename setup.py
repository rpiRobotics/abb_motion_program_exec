from setuptools import setup

setup(
    name='abb_motion_program_exec',
    version='0.2.0',
    description='Simple module to execute motion commands on ABB robots and log results',
    url='https://github.com/johnwason/abb_motion_program_exec',
    py_modules=['abb_motion_program_exec_client'],
    install_requires=[
        'bs4',
        'requests',
        'numpy'
    ]
)