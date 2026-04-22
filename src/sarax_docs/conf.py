"""Minimal Sphinx configuration for the sarax_docs book.

Build:
    pip install sphinx myst-parser breathe furo
    doxygen Doxyfile
    sphinx-build -b html source build/html
"""
project = "SARAX+ Control Stack"
author = "Ayham"
copyright = "2026, Ayham"
extensions = ["myst_parser", "breathe", "sphinx.ext.mathjax"]
source_suffix = {".rst": "restructuredtext", ".md": "markdown"}
master_doc = "index"
html_theme = "furo"
breathe_projects = {"sarax": "build/doxygen/xml"}
breathe_default_project = "sarax"
