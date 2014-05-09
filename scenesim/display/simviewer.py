#!/usr/bin/env python
"""
``scenesim.display.simviewer``
==============================

Class for visualizing simulations.

"""
from panda3d.core import (AmbientLight, AntialiasAttrib, BitMask32,
                          NodePath, PerspectiveLens, Spotlight)
from scenesim.display.lightbase import LightBase


class SimViewer(LightBase):
    """ Visualizes simulation."""

    def __init__(self):
        super(SimViewer, self).__init__()
        # Make a window.
        size = (700, 520)
        self.create_output(size, "SimViewer")
        self.output.setClearColor((0.0, 0.0, 0.0, 1.0))
        # Lights node
        self.lights = NodePath('lights')
        # Create a spotlight
        slight = Spotlight('slight')
        slight.setScene(self.root)
        slight.setShadowCaster(True, 2 ** 11, 2 ** 11)
        # Set shadow mask, so we can exclude objects from casting shadows
        self.shadow_mask = BitMask32.bit(2)
        slight.setCameraMask(self.shadow_mask)
        c = 1.4
        slight.setColor((c, c, c, 1.0))
        slight.getLens().setNearFar(4, 100)
        slight.getLens().setFov(45)
        slnp = self.lights.attachNewNode(slight)
        slnp.setPos((7, 10, 40))
        slnp.lookAt(2, 0, 1.5)
        self.root.setLight(slnp)
        # Create an ambient light.
        alight = AmbientLight('alight')
        c = 0.6
        alight.setColor((c, c, c, 1.0))
        alnp = self.lights.attachNewNode(alight)
        self.root.setLight(alnp)
        self.lights.reparentTo(self.root)
        # Set auto shading for shadows.
        self.root.setShaderAuto()
        # Set antialiasing on.
        self.root.setAntialias(AntialiasAttrib.MAuto)
        # Camera.
        lens = PerspectiveLens()
        self.lens = lens
        self.lens.setNearFar(0.1, 1000.)
        self.lens.setFov((40, 30))
        self.cameras = self.root.attachNewNode('cameras')
        self.camera = self.make_camera(self.output, lens=self.lens)
        self.camera.setPos(15, 44, 3.)
        self.camera.setPos(15, 35, 15.)
        self.camera.lookAt(0, 0, 1.)

    def create_output(self, size, name):
        self.output = self.make_window(size=size, name=name)
        self.render_frame()
        self.render_frame()


class SimImager(SimViewer):
    """ Make images of simulation scenes."""

    def create_output(self, size, name):
        self.output, tex = self.make_texture_buffer(
            size=size, name=name, mode="RTMCopyRam")
        self.render_frame()
        self.render_frame()
