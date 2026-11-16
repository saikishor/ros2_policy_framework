# Sphinx configuration: `sphinx-build -b html docs docs/_build/html` (needs Graphviz's `dot`).
project = "ros2_policy_framework"
copyright = "2026, PAL Robotics S.L."
extensions = ["sphinx.ext.graphviz"]
graphviz_output_format = "svg"
exclude_patterns = ["_build"]
html_theme = "sphinx_rtd_theme"  # pip install sphinx-rtd-theme
html_theme_options = {"navigation_depth": 3}
