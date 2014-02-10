from panda3d.core import loadPrcFile
from path import path


# The root local path.
ROOT_PATH = path(__path__[0]).abspath().split()[0]
# Load scenesim-specific Panda3d settings.
config_pth = path.joinpath(ROOT_PATH, "cfg", "Config.prc")
if config_pth.isfile():
    loadPrcFile(config_pth)
