import argparse
import os
from .project_setup import project_setup


def main():
    parser = argparse.ArgumentParser(description="Set up a new RobotStudio project for use with abb_motion_program_exec")

    parser.add_argument("project", help="Name of project to configure")
    parser.add_argument("-e","--egm", action="store_true", help="Use Externally Guided Motion")
    parser.add_argument("-d","--dir", metavar="directory", action="store", help="Directory of project if not in default location")
    parser.add_argument("-i","--inplace", action="store_true", help="Use this option if script is run within project directory. If used, the project name and argument -d are ignored; however, a dummy project name is still required.")
    parser.set_defaults(func=project_setup)

    args = parser.parse_args()
    args.func(args)

if __name__ == "__main__":
    main()
