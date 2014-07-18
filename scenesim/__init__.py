from panda3d.core import Filename, loadPrcFile
from path import path


# The root local path.
ROOT_PATH = path(__path__[0]).abspath().split()[0]
# Load scenesim-specific Panda3d settings.
config0_pth = path.joinpath(ROOT_PATH, "cfg", "Config0.prc")
config_pth = path.joinpath(ROOT_PATH, "cfg", "Config.prc")
if config_pth.isfile():
    cp = Filename.fromOsSpecific(config_pth)
    cp.makeTrueCase()
    loadPrcFile(cp)
elif config0_pth.isfile():
    cp = Filename.fromOsSpecific(config0_pth)
    cp.makeTrueCase()
    loadPrcFile(cp)
    