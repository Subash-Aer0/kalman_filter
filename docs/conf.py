"""Sphinx configuration for ESKF documentation."""

project = "ESKF with Lie Groups"
copyright = "2025, Your Name"
author = "Your Name"

release = "0.1.0"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
    "sphinx.ext.mathjax",
]

templates_path = ["_templates"]
exclude_patterns = ["_build"]

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
