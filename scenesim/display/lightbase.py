""" Class definition for LightBase. """

from panda3d.core import Loader
# from direct.showbase import Loader
from libpanda import BitMask32
import numpy as np
import os
from pandac.PandaModules import (
    AmbientLight, Camera, Filename, FrameBufferProperties, GraphicsEngine,
    GraphicsOutput, GraphicsPipe, GraphicsPipeSelection, ModelNode, NodePath,
    PerspectiveLens, PointLight, RescaleNormalAttrib, Texture,
    WindowProperties)
import sys
#
from pdb import set_trace as BP


class LightBase(object):
    """ LightBase is a lightweight interface for rendering with
    Panda3d. It contains several key Panda3d objects that are required
    for rendering, like GraphicsEngine, GraphicsPipe, GraphicsOutput,
    a root NodePath (rootnode), cameras and lights. It provides also
    methods for manipulating and reading them out them."""

    def __init__(self):
        self.init_graphics()
        self.setup_rootnode()
        self.output_list = []
        self.gsg_list = []
        self.cameras = None

    def init_graphics(self):
        """ Creates GraphicsEngine, GraphicsPipe, and loader """
        # Get a handle to the graphics pipe selector
        selection = GraphicsPipeSelection.getGlobalPtr()
        # Check for DISPLAY
        if "DISPLAY" in os.environ:
            # Use the first option (should be glx)
            pipe_type = selection.getPipeTypes()[0]
        else:
            # Use the last option (should be some fallback module)
            pipe_type = selection.getPipeTypes()[-1]
        # Create the graphics pipe
        self.pipe = selection.makePipe(pipe_type)
        # Get the graphics engine
        self.engine = GraphicsEngine.getGlobalPtr()
        # Get the model loader object and assign it to the engine
        # self.loader = Loader.Loader(self)
        # self.engine.setDefaultLoader(self.loader.loader)
        self.loader = Loader.getGlobalPtr()
        self.engine.setDefaultLoader(self.loader)

    @staticmethod
    def init_fbp():
        """ Initial / default FrameBufferProperties """
        fbp = FrameBufferProperties()
        fbp.setRgbColor(1)
        fbp.setColorBits(1)
        fbp.setAlphaBits(1)
        fbp.setDepthBits(1)
        return fbp

    @staticmethod
    def init_wp(window_type, size):
        """ Initial / default WindowProperties """
        if window_type == 'onscreen':
            wp = WindowProperties.getDefault()
            wp.setSize(size[0], size[1])
        elif window_type == 'offscreen':
            wp = WindowProperties.size(size[0], size[1])
        return wp

    def make_window(self, size=None, name=None, sort=0, xflags=0,
                    host_out=None):
        """ Create an onscreen window -- high-level interface for
        makeOutput"""

        # Handle size
        if size is None and host_out is not None:
            size = (host_out.getFbXSize(), host_out.getFbYSize())
        elif size is None:
            raise ValueError("Size not available.")

        # Initialize WindowProperties
        wp = self.init_wp('onscreen', size)

        # Create window
        output = self.make_output('onscreen', name, sort, wp=wp,
                                  xflags=xflags, host_out=host_out)

        # Handle failure to create window (returns None)
        if output is None:
            raise StandardError("Window creation failed, not sure why.")

        return output

    def make_buffer(self, size=None, name=None, sort=10, xflags=0,
                    host_out=None):
        """ Makes an offscreen buffer -- high-level interface for
        make_output"""

        # Handle size
        if size is None and host_out is not None:
            size = (host_out.getFbXSize(), host_out.getFbYSize())
        elif size is None:
            raise ValueError("Size not available.")

        # Initialize WindowProperties
        wp = self.init_wp('offscreen', size)

        # Create the buffer
        output = self.make_output('offscreen', name, sort, wp=wp,
                                  xflags=xflags, host_out=host_out)

        # Handle case when offscreen buffer cannot be directly created
        # (currently this is what happens)
        if output is None:
            print("Direct offscreen buffer creation failed...")
            print("... falling back to onscreen --> offscreen method.")

            # Open an onscreen window first, just to get a GraphicsOutput
            # object which is needed to open an offscreen buffer.
            dummywin = self.make_window(size, 'dummy_onscreen_win', sort,
                                        xflags, host_out)
            # Now, make offscreen buffer through win
            output = self.make_output('offscreen', name, sort, xflags=xflags,
                                      host_out=dummywin)

            # Handle failure to create window (returns None)
            if output is None:
                raise StandardError("Failed to create offscreen buffer.")

        return output

    def make_texture_buffer(self, size=None, name=None, sort=10,
                            xflags=0, mode=None, bitplane=None, host_out=None):
        """ Makes an offscreen buffer and adds a render texture """

        # Make the offscreen buffer
        output = self.make_buffer(size, name, sort, xflags, host_out)
        # Add the texture
        tex = self.add_render_texture(output, mode, bitplane)
        # Cause necessary stuff to be created (buffers, textures etc)
        self.render_frame()

        return output, tex

    def make_output(self, window_type, name=None, sort=10, fbp=None, wp=None,
                    xflags=0, host_out=None):
        """ Makes a GraphicsOutput object and stores it in
        self.output_list. This is the low-level interface."""

        # Input handling / defaults
        if name is None:
            name = window_type + '_win'
        if fbp is None:
            # Initialize FramebufferProperties
            fbp = self.init_fbp()
        if wp is None and host_out is not None:
            # Initialize WindowProperties
            wp = self.init_wp(window_type, (host_out.getFbXSize(),
                                            host_out.getFbYSize()))
        elif wp is None:
            raise ValueError("Size not available in either wp or win.")
        flags = xflags | GraphicsPipe.BFFbPropsOptional
        # flags' window_type switch
        if window_type == 'onscreen':
            flags = flags | GraphicsPipe.BFRequireWindow
        elif window_type == 'offscreen':
            flags = flags | GraphicsPipe.BFRefuseWindow

        # Make the window / buffer
        engine = self.engine
        pipe = self.pipe
        if host_out is None:
            output = engine.makeOutput(pipe, name, sort, fbp, wp, flags)
        else:
            output = engine.makeOutput(pipe, name, sort, fbp, wp, flags,
                                       host_out.getGsg(), host_out)

        # Add output to this instance's list
        if output is not None:
            self.add_to_output_gsg_lists(output)

            # Set background color to black by default
            output.setClearColor((0.0, 0.0, 0.0, 0.0))

            # Cause necessary stuff to be created (buffers, textures etc)
            self.render_frame()

        return output

    def add_to_output_gsg_lists(self, output):
        """ Adds the output and Gsg to the instance's running list. """
        self.output_list.append(output)
        self.gsg_list.append(output.getGsg())

    def remove_from_output_gsg_lists(self, output):
        """ Removes the output and Gsg from the instance's running list. """
        self.output_list.remove(output)
        self.gsg_list.remove(output.getGsg())

    @staticmethod
    def add_render_texture(output, mode=None, bitplane=None):
        """ Similar to GraphicsOutput's addRenderTexture.

        ** Possible mode values **
        GraphicsOutput.RTMNone
        GraphicsOutput.RTMBindOrCopy
        GraphicsOutput.RTMCopyTexture
        GraphicsOutput.RTMCopyRam
        GraphicsOutput.RTMTriggeredCopyTexture
        GraphicsOutput.RTMTriggeredCopyRam

        ** Possible bitplane values **
        GraphicsOutput.RTPStencil
        GraphicsOutput.RTPDepthStencil
        GraphicsOutput.RTPColor
        GraphicsOutput.RTPAuxRgba0
        GraphicsOutput.RTPAuxRgba1
        GraphicsOutput.RTPAuxRgba2
        GraphicsOutput.RTPAuxRgba3
        GraphicsOutput.RTPAuxHrgba0
        GraphicsOutput.RTPAuxHrgba1
        GraphicsOutput.RTPAuxHrgba2
        GraphicsOutput.RTPAuxHrgba3
        GraphicsOutput.RTPAuxFloat0
        GraphicsOutput.RTPAuxFloat1
        GraphicsOutput.RTPAuxFloat2
        GraphicsOutput.RTPAuxFloat3
        GraphicsOutput.RTPDepth
        GraphicsOutput.RTPCOUNT

        """

        if mode is None:
            mode = GraphicsOutput.RTMBindOrCopy
        elif isinstance(mode, str):
            mode = getattr(GraphicsOutput, mode)

        if bitplane is None:
            bitplane = GraphicsOutput.RTPColor
        elif isinstance(bitplane, str):
            bitplane = getattr(GraphicsOutput, bitplane)

        tex = Texture()
        tex.setFormat(Texture.FLuminance)

        # Add the texture to the buffer
        output.addRenderTexture(tex, mode, bitplane)
        # Get a handle to the texture
        assert tex == output.getTexture(), "Texture wasn't created properly."

        return tex

    def close_output(self, output):
        """
        Closes the indicated output and removes it from the list.
        """
        output.setActive(False)

        # First, remove all of the cameras associated with display
        # regions on the window.
        num_regions = output.getNumDisplayRegions()
        for i in range(num_regions):
            dr = output.getDisplayRegion(i)
            dr.setCamera(NodePath())

        # Remove this output from the list
        self.remove_from_output_gsg_lists(output)
        # Now we can actually close the window.
        engine = output.getEngine()
        engine.removeWindow(output)
        # Give the window a chance to actually close before continuing.
        engine.renderFrame()

    def close_all_outputs(self):
        """ Closes all of this instance's outputs """

        # Close them each
        for output in self.output_list:
            self.close_output(output)
        # Clear the output list
        self.output_list = []

    def make_camera(self, output, sort=0, dr_dims=(0, 1, 0, 1),
                    aspect_ratio=None, clear_depth=False, clear_color=None,
                    lens=None, cam_name='camera0', mask=None):
        """
        Makes a new 3-d camera associated with the indicated window,
        and creates a display region in the indicated subrectangle.

        If stereo is True, then a stereo camera is created, with a
        pair of DisplayRegions.  If stereo is False, then a standard
        camera is created.  If stereo is None or omitted, a stereo
        camera is created if the window says it can render in stereo.

        If useCamera is not None, it is a NodePath to be used as the
        camera to apply to the window, rather than creating a new
        camera.
        """

        # self.cameras is the parent node of all cameras: a node that
        # we can move around to move all cameras as a group.
        if self.cameras is None:
            # We make it a ModelNode with the PTLocal flag, so that a
            # wayward flatten operations won't attempt to mangle the
            # camera.
            self.cameras = self.rootnode.attachNewNode(ModelNode('cameras'))
            self.cameras.node().setPreserveTransform(ModelNode.PTLocal)

        # Make a new Camera node.
        cam_node = Camera(cam_name)
        if lens is None:
            lens = PerspectiveLens()
            if aspect_ratio is None:
                aspect_ratio = self.get_aspect_ratio(output)
            lens.setAspectRatio(aspect_ratio)
            lens.setNear(0.1)
            lens.setFar(1000.0)
        if lens is not None:
            cam_node.setLens(lens)
        camera = self.cameras.attachNewNode(cam_node)

        # Masks out part of scene from camera
        if mask is not None:
            if (isinstance(mask, int)):
                mask = BitMask32(mask)
            cam_node.setCameraMask(mask)

        # Make a display region
        dr = output.makeDisplayRegion(*dr_dims)
        # By default, we do not clear 3-d display regions (the entire
        # window will be cleared, which is normally sufficient).  But
        # we will if clearDepth is specified.
        if clear_depth:
            dr.setClearDepthActive(1)
        if clear_color:
            dr.setClearColorActive(1)
            dr.setClearColor(clear_color)
        dr.setSort(sort)
        dr.setCamera(camera)
        dr.setActive(True)
        return camera

    @staticmethod
    def get_aspect_ratio(output):
        """ Returns the actual aspect ratio of the indicated (or main
         window), or the default aspect ratio if there is not yet a
         main window."""

        aspect_ratio = 1
        if output.hasSize():
            aspect_ratio = float(output.getSbsLeftXSize()) / \
                           float(output.getSbsLeftYSize())
        else:
            wp = output.getRequestedProperties()
            if not wp.hasSize():
                wp = WindowProperties.getDefault()
            aspect_ratio = float(wp.getXSize()) / float(wp.getYSize())
        return aspect_ratio

    @staticmethod
    def make_lights():
        """ Create one point light and an ambient light."""
        lights = NodePath('lights')

        # Create point lights
        plight = PointLight('plight1')
        light = lights.attachNewNode(plight)
        light.setPos((3, -10, 2))
        light.lookAt(0, 0, 0)

        # Create ambient light
        alight = AmbientLight('alight')
        alight.setColor((0.75, 0.75, 0.75, 1.0))
        lights.attachNewNode(alight)

        return lights

    def setup_rootnode(self):
        """
        Creates the rootnode scene graph, the primary scene graph for
        rendering 3-d geometry.
        """
        self.rootnode = NodePath('rootnode')
        self.rootnode.setAttrib(RescaleNormalAttrib.makeDefault())
        self.rootnode.setTwoSided(0)
        self.backface_culling_enabled = 1
        self.texture_enabled = 1
        self.wireframe_enabled = 0

    def wireframe_on(self):
        """ Toggle wireframe mode on."""
        self.rootnode.setRenderModeWireframe(100)
        self.rootnode.setTwoSided(1)
        self.wireframe_enabled = 1

    def wireframe_off(self):
        """ Toggle wireframe mode off."""
        self.rootnode.clearRenderMode()
        self.rootnode.setTwoSided(not self.backface_culling_enabled)
        self.wireframe_enabled = 0

    @staticmethod
    def trigger_copy(output):
        """ This signals the texture to be pushed to RAM after the
        next renderFrame"""
        output.trigger_copy()

    def render_frame(self):
        """ Render the frame."""
        self.engine.renderFrame()
        # If you're trying to read the texture buffer, remember to
        # call self.trigger_copy()

    @staticmethod
    def get_tex_image(tex, freshape=True):
        """ Returns numpy arr containing image in tex """

        # Remember to call self.triggerCopy() before
        # self.renderFrame(), or the next frame won't be pushed to RAM

        if not tex.hasRamImage():
            arr = None
        else:
            size = (tex.getXSize(), tex.getYSize())
            texdata = tex.getRamImage().getData()
            arr = np.fromstring(texdata, 'u1')

            if freshape:
                arr = np.reshape(arr, list(size) + [-1])[:, :, [2, 1, 0, 3]]

        return arr

    @staticmethod
    def screenshot(output, pth=None):
        """ Similar to ShowBase's screenshot """
        if pth is None:
            filename = GraphicsOutput.makeScreenshotFilename()
        else:
            filename = Filename(pth)

        if isinstance(output, GraphicsOutput):
            f_success = output.saveScreenshot(filename)
        elif isinstance(output, Texture):
            if output.getZSize() > 1:
                f_success = output.write(filename, 0, 0, 1, 0)
            else:
                f_success = output.write(filename)
        else:
            raise TypeError('Unhandled output type: ' + type(output))

        return f_success

    def destroy(self):
        """ self.__exitfunc() calls this automatically """
        self.close_all_outputs()
        if getattr(self, 'loader', None):
            self.loader.destroy()
            self.loader = None
        if getattr(self, 'engine', None):
            self.engine.removeAllWindows()
            self.engine = None
        if getattr(self, 'pipe', None):
            self.pipe = None

    def __exitfunc(self):
        """ Customize pre-exit commands."""
        self.destroy()

    def user_exit(self):
        """ The user has requested we exit the program. """
        self.__exitfunc()
        sys.exit()

    @staticmethod
    def destroy_windows():
        """ General destroy windows method."""
        GraphicsEngine.getGlobalPtr().removeAllWindows()
