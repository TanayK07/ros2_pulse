# Sphinx settings for the ros2_pulse guide, built by rosdoc2 for docs.ros.org.
#
# rosdoc2 runs this file from inside its own conf.py, after it has set the project name,
# version, theme (sphinx_rtd_theme) and extensions, so only the additions live here. Only
# extensions rosdoc2 itself installs are available on the build farm: myst_parser and the
# ones shipped with Sphinx.

# rosdoc2 copies the whole repository root's standard documents (README, CHANGELOG, ...) next
# to this guide. The README is written for GitHub and links into the repository, and the guide
# covers the same ground, so only the changelog, license and contributing pages are used.
exclude_patterns = [
    'README.md',
    '__README.rst',
    '__readme_include.rst',
    '__standards.rst',
    '__PACKAGE.rst',
    'CHANGELOG.md',
    'CONTRIBUTING.md',
]

myst_enable_extensions = [
    'colon_fence',
    'deflist',
]
myst_heading_anchors = 3

html_static_path = ['_static']
html_css_files = ['ros2_pulse.css']
