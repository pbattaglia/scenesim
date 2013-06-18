#!/usr/bin/env python
from distutils.core import setup


root_pkg = "scenesim"
sub_pkgs = ["cfg", "display", "lib", "objects", "physics", "tests"]
packages = [root_pkg] + ["%s.%s" % (root_pkg, sub_pkg) for sub_pkg in sub_pkgs]


setup(name="scenesim",
      version="0.3",
      description="Create scenes and simulate their physics and graphics.",
      author="Peter Battaglia",
      author_email="pbatt@mit.edu",
      url="http://scenesim.org",
      packages=packages,
      requires = ["Polygon2", "networkx", "numpy", "panda3d", "path.py"])
