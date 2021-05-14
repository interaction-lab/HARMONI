# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys

# import catkin_pkg.package

# catkin_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# catkin_package = catkin_pkg.package.parse_package(
#     os.path.join(catkin_dir, catkin_pkg.package.PACKAGE_MANIFEST_FILENAME)
# )
sys.path.insert(0, os.path.abspath("../"))


# -- Project information -----------------------------------------------------

project = "HARMONI"
copyright = "2021, Chris, Micol, Michael"
author = "Chris, Micol, Michael"


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
source_suffix = [".rst", ".md"]
master_doc = "index"
extensions = [
    "sphinx.ext.autodoc",  # Core library for html generation from docstrings
    "sphinx.ext.autosummary",  # Create neat summary tables
    "myst_parser",
    "sphinx.ext.napoleon",
    "autoapi.extension",
]
autoapi_type = "python"
autoapi_dirs = [
    "../harmoni_actuators",
    "../harmoni_core",
    "../harmoni_detectors",
    "../harmoni_dialogues",
    "../harmoni_sensors",
]
autoapi_add_toctree_entry = True
autosummary_generate = True  # Turn on sphinx.ext.autosummary
# autoapi_root = "api-link"
# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]
autoapi_template_dir = "_autoapi_templates"
# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]
autoapi_ignore = ["*/test*"]

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
# html_static_path = ["_static"]
