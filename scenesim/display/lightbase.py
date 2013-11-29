"""
``scenesim.display.lightbase``
==============================

Class definition for LightBase.

"""
import atexit
import os
import sys
##
from libpanda import BitMask32
from numpy import fromstring
from panda3d.core import Loader as PandaLoader
from panda3d.core import (AmbientLight, Camera, Filename,
                          FrameBufferProperties, GraphicsEngine,
                          GraphicsOutput, GraphicsPipe, GraphicsPipeSelection,
                          ModelNode, NodePath, PNMImage, PerspectiveLens,
                          PointLight, RescaleNormalAttrib, Texture,
                          TexturePool, WindowProperties)
from path import path


class Loader(object):
    ## TODO:
    #
    # Make Loader than can load models and textures:
    #
    # PandaLoader.loadAsync
    # TexturePool.loadTexture

    panda_loader = PandaLoader.getGlobalPtr()
    texture_loader = TexturePool
    load_model = panda_loader.loadSync
    load_texture = texture_loader.loadTexture
    loadSync = load_model
    loadModel = load_model


class LightBase(object):
    """ LightBase is a lightweight interface for rendering with
    Panda3d. It contains several key Panda3d objects that are required
    for rendering, like GraphicsEngine, GraphicsPipe, GraphicsOutput,
    a root NodePath (root), cameras and lights. It provides also
    methods for manipulating and reading them out them."""

    def __init__(self):
        self.init_graphics()
        self.setup_root()
        self.output_list = []
        self.gsg_list = []
        self.cameras = None
        atexit.register(self._exitfunc)

    def init_graphics(self):
        """Creates GraphicsEngine, GraphicsPipe, and loader."""
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
        self.loader = Loader()
        self.engine.setDefaultLoader(self.loader.panda_loader)

    @staticmethod
    def init_fbp():
        """Initial / default FrameBufferProperties.

        Return:
            (FrameBufferProperties): FrameBufferProperties object.

        """
        fbp = FrameBufferProperties()
        fbp.setRgbColor(1)
        fbp.setColorBits(1)
        fbp.setAlphaBits(1)
        fbp.setDepthBits(1)
        return fbp

    @staticmethod
    def init_wp(window_type, size):
        """Initial / default WindowProperties.

        Args:
            window_type (str): Window type.
            size (Iterable, 2): Width, height.

        Return:
            (WindowProperties): WindowProperties object.

        """
        if window_type == "onscreen":
            wp = WindowProperties.getDefault()
            wp.setSize(size[0], size[1])
        elif window_type == "offscreen":
            wp = WindowProperties.size(size[0], size[1])
        return wp

    def make_window(self, size=None, name=None, sort=0, xflags=0,
                    host_out=None):
        """Create an onscreen window. High-level interface for `make_output`

        Keyword Args:
            size (Iterable, 2): Width, height.
            name (str): Window name.
            sort (int): Sort order.
            xflags (int): GraphicsPipe bit flags.
            host_out (GraphicsOutput): Output object.

        Return:
            (GraphicsOutput): Output object.

        """
        # Set size.
        if size is None and host_out is not None:
            size = (host_out.getFbXSize(), host_out.getFbYSize())
        elif size is None:
            raise ValueError("Size not available.")
        # Initialize WindowProperties.
        wp = self.init_wp("onscreen", size)
        # Create window.
        output = self.make_output("onscreen", name, sort, wp=wp,
                                  xflags=xflags, host_out=host_out)
        # Handle failure to create window (returns None)
        if output is None:
            raise StandardError("Window creation failed, not sure why.")
        return output

    def make_buffer(self, size=None, name=None, sort=10, xflags=0,
                    host_out=None):
        """Create an offscreen buffer. High-level interface for
        `make_output`.

        Keyword Args:
            size (Iterable, 2): Width, height.
            name (str): Window name.
            sort (int): Sort order.
            xflags (int): GraphicsPipe bit flags.
            host_out (GraphicsOutput): Output object.

        Return:
            (GraphicsOutput): Output object.

        """
        # Set size.
        if size is None and host_out is not None:
            size = (host_out.getFbXSize(), host_out.getFbYSize())
        elif size is None:
            raise ValueError("Size not available.")
        # Initialize WindowProperties
        wp = self.init_wp("offscreen", size)
        # Create the buffer
        output = self.make_output("offscreen", name, sort, wp=wp,
                                  xflags=xflags, host_out=host_out)
        # Handle case when offscreen buffer cannot be directly created
        # (currently this is what happens)
        if output is None:
            print("Direct offscreen buffer creation failed...")
            print("... falling back to onscreen --> offscreen method.")
            # Open an onscreen window first, just to get a GraphicsOutput
            # object which is needed to open an offscreen buffer.
            dummywin = self.make_window(size, "dummy_onscreen_win", sort,
                                        xflags, host_out)
            # Now, make offscreen buffer through win
            output = self.make_output("offscreen", name, sort, xflags=xflags,
                                      host_out=dummywin)
            # Handle failure to create window (returns None)
            if output is None:
                raise StandardError("Failed to create offscreen buffer.")
        return output

    def make_texture_buffer(self, size=None, name=None, sort=10,
                            xflags=0, mode=None, bitplane=None, host_out=None):
        """Makes an offscreen buffer and adds a render texture.

        Keyword Args:
            size (Iterable, 2): Width, height.
            name (str): Window name.
            sort (int): Sort order.
            xflags (int): GraphicsPipe bit flags.
            mode (GraphicsOutput.RenderTextureMode): see
                :py:meth:`add_render_texture` for possible values.
            bitplane (DrawableRegion.RenderTexturePlane): see
                :py:meth:`add_render_texture` for possible values.
            host_out (GraphicsOutput): Output object.

        Return:
            (GraphicsOutput): Output object.
            (Texture): Texture object.

        """
        # Make the offscreen buffer
        output = self.make_buffer(size, name, sort, xflags, host_out)
        # Add the texture
        tex = self.add_render_texture(output, mode, bitplane)
        # Cause necessary stuff to be created (buffers, textures etc)
        self.render_frame()
        return output, tex

    def make_output(self, window_type, name=None, sort=10, fbp=None, wp=None,
                    xflags=0, host_out=None):
        """Create a GraphicsOutput object and store in self.output_list.
        This is the low-level interface.

        Args:
            window_type (str): Window type.

        Keyword Args:
            name (str): Window name.
            sort (int): Sort order.
            fbp (FrameBufferProperties): FrameBufferProperties object.
            wp (WindowProperties): WindowProperties object.
            xflags (int): GraphicsPipe bit flags.
            host_out (GraphicsOutput): Output object.

        Return:
            (GraphicsOutput): Output object.

        """
        # Input handling / defaults
        if name is None:
            name = window_type + "_win"
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
        if window_type == "onscreen":
            flags = flags | GraphicsPipe.BFRequireWindow
        elif window_type == "offscreen":
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
        """Adds the `output` and GSG to the instance's running list.

        Args:
            output (GraphicsOutput): Graphics output.

        """
        self.output_list.append(output)
        self.gsg_list.append(output.getGsg())

    def remove_from_output_gsg_lists(self, output):
        """Removes the `output` and GSG from the instance's running list.

        Args:
            output (GraphicsOutput): Graphics output.

        """
        self.output_list.remove(output)
        self.gsg_list.remove(output.getGsg())

    @staticmethod
    def add_render_texture(output, mode=None, bitplane=None):
        """Add render texture to `output`.

        Args:
            output (GraphicsOutput): Graphics output.

        Keyword Args:
            mode (GraphicsOutput.RenderTextureMode):
                | RTMNode
                | RTMBindOrCopy
                | RTMCopyTexture
                | RTMCopyRam
                | RTMTriggeredCopyTexture
                | RTMTriggeredCopyRam
            bitplane (DrawableRegion.RenderTexturePlane):
                | RTPStencil
                | RTPDepthStencil
                | RTPColor
                | RTPAuxRgba0
                | RTPAuxRgba1
                | RTPAuxRgba2
                | RTPAuxRgba3
                | RTPAuxHrgba0
                | RTPAuxHrgba1
                | RTPAuxHrgba2
                | RTPAuxHrgba3
                | RTPAuxFloat0
                | RTPAuxFloat1
                | RTPAuxFloat2
                | RTPAuxFloat3
                | RTPDepth
                | RTPCOUNT

        Return:
            (Texture): Texture object.

        """
        # Mode.
        if mode is None:
            mode = GraphicsOutput.RTMBindOrCopy
        elif isinstance(mode, str):
            mode = getattr(GraphicsOutput, mode)
        if bitplane is None:
            bitplane = GraphicsOutput.RTPColor
        elif isinstance(bitplane, str):
            bitplane = getattr(GraphicsOutput, bitplane)
        # Bitplane.
        if bitplane is GraphicsOutput.RTPColor:
            fmt = Texture.FLuminance
        elif bitplane is GraphicsOutput.RTPDepth:
            fmt = Texture.FDepthComponent
        # Get a handle to the texture.
        tex = Texture()
        tex.setFormat(fmt)
        # Add the texture to the buffer.
        output.addRenderTexture(tex, mode, bitplane)
        tex.clearRamImage()
        return tex

    def close_output(self, output):
        """Closes the indicated `output` and removes it from the list.

        Args:
            output (GraphicsOutput): Graphics output.

        """
        output.setActive(False)
        # First, remove all of the cameras associated with display
        # regions on the window.
        num_regions = output.getNumDisplayRegions()
        for i in range(num_regions):
            dr = output.getDisplayRegion(i)
            dr.setCamera(NodePath())
        # Remove this output from the list.
        self.remove_from_output_gsg_lists(output)
        # Now we can actually close the window.
        engine = output.getEngine()
        engine.removeWindow(output)
        # Give the window a chance to actually close before continuing.
        engine.renderFrame()

    def close_all_outputs(self):
        """Closes all of this instance's outputs."""
        for output in self.output_list:
            self.close_output(output)
        # Clear the output list
        self.output_list = []

    def make_camera(self, output, sort=0, dr_dims=(0, 1, 0, 1),
                    aspect_ratio=None, clear_depth=False, clear_color=None,
                    lens=None, cam_name="camera0", mask=None):
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

        Args:
            output (GraphicsOutput): Output object.

        Keyword Args:
            sort (int): Sort order.
            dr_dims (Iterable, 4): DisplayRegion dimensions.
            aspect_ratio (float): Aspect ratio.
            clear_depth (bool): Indicator to clear depth buffer.
            clear_color (bool): Indicator to clear color buffer.
            lens (Lens): Lens object.
            cam_name (str): Window name.
            mask (BitMask32): Bit mask that indicates which objects to render.

        Return:
            (NodePath): Camera nodepath.

        """

        # self.cameras is the parent node of all cameras: a node that
        # we can move around to move all cameras as a group.
        if self.cameras is None:
            # We make it a ModelNode with the PTLocal flag, so that a
            # wayward flatten operations won't attempt to mangle the
            # camera.
            self.cameras = self.root.attachNewNode(ModelNode("cameras"))
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
        """Returns aspect ratio of `output`'s window, or default aspect
        ratio if it has no window.

        Args:
            output (GraphicsOutput): Graphics output.

        Return:
            (float): Aspect ratio.

        """
        aspect_ratio = 1
        if output.hasSize():
            aspect_ratio = (float(output.getSbsLeftXSize()) /
                            float(output.getSbsLeftYSize()))
        else:
            wp = output.getRequestedProperties()
            if not wp.hasSize():
                wp = WindowProperties.getDefault()
            aspect_ratio = float(wp.getXSize()) / float(wp.getYSize())
        return aspect_ratio

    @staticmethod
    def make_lights():
        """Create one point light and an ambient light.

        Return:
           (NodePath): Lights nodepath.

        """
        lights = NodePath("lights")
        # Create point lights.
        plight = PointLight("plight1")
        light = lights.attachNewNode(plight)
        light.setPos((3, -10, 2))
        light.lookAt(0, 0, 0)
        # Create ambient light.
        alight = AmbientLight("alight")
        alight.setColor((0.75, 0.75, 0.75, 1.0))
        lights.attachNewNode(alight)
        return lights

    def setup_root(self):
        """Set up the scene graph."""
        self.root = NodePath("root")
        self.root.setAttrib(RescaleNormalAttrib.makeDefault())
        self.root.setTwoSided(False)
        self.backface_culling_enabled = True
        self.texture_enabled = True
        self.wireframe = False

    @property
    def wireframe(self):
        """(Property) Get wireframe mode.

        Return:
            (bool): Indicates wireframe ON.

        """
        return self._wireframe

    @wireframe.setter
    def wireframe(self, val):
        """(Property) Set wireframe mode.

        Args:
            val (bool): Indicates wireframe ON.

        """
        if self.wireframe:
            self.root.clearRenderMode()
            self.root.setTwoSided(not self.backface_culling_enabled)
        else:
            self.root.setRenderModeWireframe(100)
            self.root.setTwoSided(True)

    @staticmethod
    def trigger_copy(output):
        """Signals the texture to be pushed to RAM after the next
        render_frame.

        Args:
            output (GraphicsOutput): Graphics output.

        """
        output.trigger_copy()

    def render_frame(self):
        """Render the frame."""
        # If you're trying to read the texture buffer, remember to
        # call self.trigger_copy()
        self.engine.renderFrame()

    @staticmethod
    def get_tex_array(tex, reshape=True):
        """Returns image (as ndarray) in `tex`.

        Args:
            tex (Texture): Texture handle.

        Keyword Args:
            reshape (bool): Indicator to reshape image array to 2D.

        Return:
            (ndarray): Image array.

        """
        # Remember to call self.trigger_copy() before
        # self.render_frame(), or the next frame won't be pushed to RAM
        if not tex.hasUncompressedRamImage():
            img = None
        else:
            texel_type = tex.getComponentType()
            if texel_type == Texture.TUnsignedByte:
                dtype = "u1"
            elif texel_type == Texture.TUnsignedShort:
                dtype = "u2"
            elif texel_type == Texture.TFloat:
                dtype = "f4"
            elif texel_type == Texture.TUnsignedInt248:
                dtype = "u4"
            # texel_width = tex.getComponentWidth()
            texdata = tex.getUncompressedRamImage().getData()
            img = fromstring(texdata, dtype=dtype)
            if reshape:
                shape = [tex.getYSize(), tex.getXSize(), -1]
                n_channels = tex.getNumComponents()
                if n_channels == 4:
                    channel_order = [2, 1, 0, 3]
                elif n_channels == 3:
                    channel_order = [2, 1, 0]
                else:
                    # from pdb import set_trace as BP; BP()
                    channel_order = range(n_channels)
                img = img.reshape(shape)[:, :, channel_order]
        return img

    @staticmethod
    def get_tex_image(tex):
        """Returns image (as PNMImage) in `tex`.

        Args:
            tex (Texture): Texture handle.

        Return:
            (PNMImage): Image object.

        """
        # Remember to call self.trigger_copy() before
        # self.render_frame(), or the next frame won't be pushed to RAM.
        if not tex.hasRamImage():
            img = None
        else:
            img = PNMImage()
            tex.store(img)
        return img

    @staticmethod
    def screenshot(output, pth=None):
        """Save screenshot of `output`.

        Args:
            output (GraphicsOutput): Graphics output.

        Keyword Args:
            pth (str): Path to save screenshot.

        Return:
            (bool): Indicates successful save.

        """
        if pth is None:
            filename = GraphicsOutput.makeScreenshotFilename()
        else:
            filename = Filename(pth)
        if isinstance(output, GraphicsOutput):
            success = output.saveScreenshot(filename)
        elif isinstance(output, Texture):
            if output.getZSize() > 1:
                success = output.write(filename, 0, 0, 1, 0)
            else:
                success = output.write(filename)
        else:
            raise TypeError("Unhandled output type: " + type(output))
        return success

    def destroy(self):
        """Close outputs associated with this instance."""
        self.close_all_outputs()
        if getattr(self, "engine", None):
            self.engine.removeAllWindows()
            self.engine = None
        if getattr(self, "pipe", None):
            self.pipe = None

    def _exitfunc(self):
        """atexit function."""
        self.destroy()

    @staticmethod
    def destroy_windows():
        """Destroy all graphics windows globally."""
        GraphicsEngine.getGlobalPtr().removeAllWindows()
