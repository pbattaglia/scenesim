#!/usr/bin/env python
"""View scenes."""
import sys
##
from panda3d.core import ConfigVariableString, Vec3
from path import path
from scenesim.display.viewer import Viewer
from scenesim.objects import GSO, SSO
from scenesim.physics.bulletbase import BulletBase
##
from ipdb import set_trace as BP


def view(stim_dir):
    """View the stimuli."""
    # Change window size.
    view_config = ConfigVariableString('win-size')
    view_config.setValue('800 800')
    # Load the scenes from disk.
    scene_fns = sorted(dir_pth.files('tower*sso'))
    scenes = []
    for fn in scene_fns:
        with fn.open() as fid:
            scene = SSO.load_tree(fid)
        scenes.append(scene)
    # Load the ground.
    with path.joinpath(dir_pth, 'ground.sso').open() as fid:
        ground = SSO.load_tree(fid)
    # Set up BulletBase.
    bbase = BulletBase()
    bbase.init()
    bbase.gravity = Vec3(0., 0., -9.81 * 10)
    bbase.set_axis_constraint(1, True)
    # Setup viewer.
    viewer = Viewer()
    # Adjust the camera and lights.
    viewer.cameras.setPos(0., 7.5, 2.)
    viewer.cameras.lookAt(0, 0, 1.1)
    viewer.camLens.setFar(20)
    slnp = viewer.lights.find('slight')
    slnp.setPos((3, 4, 12))
    slnp.lookAt(0, 0, 0)
    s = 3.
    slnp.node().setColor((s, s, s, 1.))
    slnp.node().getLens().setFov(60)
    alnp = viewer.lights.find('alight')
    a = 0.2
    alnp.node().setColor((a, a, a, 1.))
    b = 0.3
    viewer.win.setClearColor((b, b, b, 0.0))
    # Initialize the physical world and objects.
    viewer.init_physics(bbase)
    viewer.init_ssos(scenes)
    viewer.init_background(ground)
    # View.
    viewer.run()


if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        dir_pth = path(args[0])
    except IndexError:
        dir_pth = path('tower_ssos')
    view(dir_pth)
