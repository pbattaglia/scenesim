#!/usr/bin/env python
from setuptools import setup


root_pkg = "scenesim"
sub_pkgs = ["cfg", "display", "objects", "physics", "tests", "util"]
packages = [root_pkg] + ["%s.%s" % (root_pkg, sub_pkg) for sub_pkg in sub_pkgs]

setup(name="scenesim",
      version="0.3",
      description="Create scenes and simulate their physics and graphics.",
      author="Peter Battaglia",
      author_email="pbatt@mit.edu",
      url="http://scenesim.org",
      packages=packages,
      install_requires=["Sphinx>=1.1.3",
                        "argparse>=1.2.1",
                        "networkx>=1.8.1",
                        "numpy>=1.8.0",
                        "numpydoc>=0.4",
                        "path.py>=5.0",
                        "sphinxcontrib-napoleon>=0.2.1",
                        "Polygon2",
                        # "https://bitbucket.org/jraedler/polygon2/downloads/Polygon2-2.0.6.zip",
                    ],
      provides=["scenesim"])
