# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'abb_motion_program_exec'
copyright = '2022, Wason Technology LLC, Rensselaer Polytechnic Institute'
author = 'John Wason'

import abb_motion_program_exec

# The short X.Y version.
# version = abb_motion_program_exec.__version__
# The full version, including alpha/beta/rc tags.
# release = abb_motion_program_exec.__version__

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
     "sphinx.ext.autodoc",
     "sphinx_autodoc_typehints",
     "recommonmark"
]

source_suffix = [".rst", ".md"]

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ['_static']

# https://www.lieret.net/2021/05/20/include-readme-sphinx/
import pathlib

# The readme that already exists
readme_path = pathlib.Path(__file__).parent.resolve().parent / "README.md"
# We copy a modified version here
readme_target = pathlib.Path(__file__).parent / "readme.md"

with readme_target.open("w") as outf:
    # Change the title to "Readme"
    outf.write(
        "\n".join(
            [
                "Readme",
                "======",
            ]
        )
    )
    lines = []
    for line in readme_path.read_text().split("\n"):
        if line.startswith("# "):
            # Skip title, because we now use "Readme"
            # Could also simply exclude first line for the same effect
            continue
        line = line.replace("docs/figures", "figures/")
        line = line.replace("docs/", "")
        lines.append(line)
    outf.write("\n".join(lines))
