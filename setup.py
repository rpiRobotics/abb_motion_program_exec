import setuptools

# read the contents of your README file
from pathlib import Path
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

setuptools.setup(
    name='abb_motion_program_exec',
    version='0.4.0',
    description='Python package to execute motion commands on ABB robots and log results',
    url='https://github.com/johnwason/abb_motion_program_exec',
    packages=setuptools.find_packages("src"),
    package_dir={"" :"src"},
    install_requires=[
        'requests',
        'numpy',
        'abb_robot_client'
    ],
    long_description=long_description,
    long_description_content_type='text/markdown'
)