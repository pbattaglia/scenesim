#!/usr/bin/env python
""" Viewer for SSOs."""
# Standard
from pdb import set_trace as BP
import sys
# External
from direct.showbase.ShowBase import ShowBase
from libpanda import AntialiasAttrib, BitMask32, PerspectiveLens, Point3, Vec3
from panda3d.bullet import BulletRigidBodyNode
from panda3d.core import WindowProperties
from pandac.PandaModules import AmbientLight, NodePath, Spotlight
from path import path
# Project
from scenesim.objects import GSO, PSO, SSO
from scenesim.physics.bulletbase import BulletBase


class Viewer(ShowBase, object):
    """ Viewer for SSOs."""

    def __init__(self):
        # ShowBase init.
        ShowBase.__init__(self)
        self.win_size = (800, 800)
        # Black background
        self.win.setClearColor((0.0, 0.0, 0.0, 1.0))
        # Set up lights.
        self.lights = NodePath("lights")
        # Spotlight. Casts shadows.
        slight = Spotlight("slight")
        slight.setScene(self.render)
        slight.setShadowCaster(True, 2 ** 11, 2 ** 11)
        # Set shadow mask, so we can exclude objects from casting shadows
        self.shadow_mask = BitMask32.bit(2)
        slight.setCameraMask(self.shadow_mask)
        slight.setColor((1.2, 1.2, 1.2, 1.))
        slight.getLens().setFov(45)
        slight.getLens().setNearFar(1, 100)
        slnp = self.lights.attachNewNode(slight)
        slnp.setPos((6, 8, 20))
        slnp.lookAt(0, 0, 0)
        self.render.setLight(slnp)
        # Ambient light.
        alight = AmbientLight("alight")
        a = 0.75
        alight.setColor((a, a, a, 1.0))
        #alight.setColor((0.8, 0.8, 0.8, 1.0))
        alnp = self.lights.attachNewNode(alight)
        self.render.setLight(alnp)
        self.lights.reparentTo(self.render)
        # Set auto shading for shadows
        self.render.setShaderAuto()
        # Set antialiasing on
        self.render.setAntialias(AntialiasAttrib.MAuto)
        # Camera
        self.camera_rot = self.render.attachNewNode("camera_rot")
        self.cameras = self.camera_rot.attachNewNode("cameras")
        self.cameras.setPos(14, 32, 9.)
        self.look_at = self.render.attachNewNode("look_at")
        self.look_at.setPos(Point3(2, 0, 1))
        self.cameras.lookAt(self.look_at)
        self.camera.reparentTo(self.cameras)
        # Adjust the camera's lens
        lens = PerspectiveLens()
        self.cam.node().setLens(lens)
        self.camLens = lens
        self.camLens.setNearFar(0.01, 1000.0)
        #
        # Initialize / set variables
        self.sso = None
        self.ssos = []
        self.cache = None
        self.scene = SSO("scene")
        self.scene.reparentTo(self.render)
        # Key callbacks.
        self.accept("escape", self.exit)
        self.accept("0", self.reset_sso)
        self.accept("arrow_left", self.prev)
        self.accept("arrow_right", self.next)
        self.accept("f1", self.toggle_debug)
        self.accept("o", self.physics_once, extraArgs=[1. / 10])
        self.accept("i", self.physics_once, extraArgs=[1. / 10000])
        # Task list: name: (key, args)
        tasks = {"physics": ("p",),
                 "repel": ("t",),
                 "bump": ("f",),
                 "rotate": ("r", 20),
                 "rotate90": ("h",),
                 "ss_task": ("s",),
                 "ssa_task": ("w",),
                 "bp": ("b",)}
        # Add tasks
        for key, val in tasks.iteritems():
            call = [key] + list(val[1:])
            self.accept(val[0], self.toggle_task, call)
        # These are the key events that we will never ignore
        self.permanent_events = self.getAllAccepting()
        # These are the key events that we will never ignore
        self.permanent_tasks = [task.getName()
                                for task in self.taskMgr.getAllTasks()]
        self.start_time = -1
        self.old_elapsed = 0

    @property
    def win_size(self):
        """ Returns window size."""
        props = WindowProperties(self.win.getProperties())
        return props.getXSize(), props.getYSize()

    @win_size.setter
    def win_size(self, wh):
        """ Sets window size."""
        props = WindowProperties(self.win.getProperties())
        props.setSize(*wh)
        self.size = wh
        self.win.requestProperties(props)

    def toggle_fullscreen(self):
        """ Toggles fullscreen mode."""
        props = WindowProperties(self.win.getProperties())
        if props.getFullscreen():
            props.setSize(*self.size)
            props.setFullscreen(False)
        else:
            w = self.pipe.getDisplayWidth()
            h = self.pipe.getDisplayHeight()
            props.setSize(w, h)
            props.setFullscreen(True)
        self.win.requestProperties(props)

    def init_physics(self, bbase):
        """ Initialize the physics resources."""
        self.bbase = bbase
        self.debug_np = self.render.attachNewNode(self.bbase.setup_debug())

    def init_ssos(self, ssos):
        """ Initialize the ssos."""
        GSO.loader = self.graphicsEngine.getDefaultLoader()
        # Put all the input ssos into one list.
        self.ssos = []
        for sso in ssos:
            if not isinstance(sso, NodePath):
                raise TypeError("Must be NodePath: %s (%s)" % (sso, type(sso)))
            # Set up the node and its descendants.
            # BP()
            sso.init_tree(tags=("model",))
            # nodes = sso.descendants()
            # for node in nodes:
            #     # Get the state tag.
            #     state = get_tag(node)
            #     # if isinstance(state, GState):
            #     #     # Set GState's loader.
            #     #     state.loader = self.graphicsEngine.getDefaultLoader()
            #     # Push the state to the node and initialize the resources.
            #     state.set_node(node)
            #     state.initialize(node, names=["model"])
            # Store the sso.
            self.ssos.append(sso)
        # Number of ssos.
        self.n_ssos = len(self.ssos)

    def init_background(self, bg):
        """ Initialize the background."""
        # Put all the input ssos into one list.
        if not isinstance(bg, NodePath):
            raise TypeError("Must be NodePath: %s (%s)" % (bg, type(bg)))
        GSO.loader = self.graphicsEngine.getDefaultLoader()
        bg.init_tree(tags=("model",))
        # # Set up the node and its descendants.
        # nodes = bg.descendants()
        # for node in nodes:
        #     # Get the state tag.
        #     state = get_tag(node)
        #     if isinstance(state, GState):
        #         # Set GState's loader.
        #         state.loader = self.loader
        #     try:
        #         # Push the state to the node and initialize the resources.
        #         state.set_node(node)
        #         state.initialize(node, names=["model"])
        #     except AttributeError:
        #         # It might just be a regular NodePath with no tag.
        #         BP()
        #         pass
        self.background = bg
        self.background.reparentTo(self.scene)

    def toggle_task(self, taskname, sort=0):
        """ Toggles taskMgr task 'taskname'."""
        if not self.taskMgr.hasTaskNamed(taskname):
            self.taskMgr.add(getattr(self, taskname), taskname, sort=sort)
            if taskname == "physics":
                self.reset_physics()
        else:
            self.taskMgr.remove(taskname)

    def reset_physics(self):
        """ Resets physics."""
        self.start_time = self.taskMgr.globalClock.getFrameTime()
        self.old_elapsed = 0.

    def physics(self, task):
        """ Task: simulate physics."""
        # Elapsed time.
        dt = self._get_elapsed() - self.old_elapsed
        # Update amount of time simulated so far.
        self.old_elapsed += dt
        # Step the physics dt time.
        size_sub = 1. / 180.
        n_subs = int(dt / size_sub)
        self.bbase.step(dt, n_subs, size_sub)
        return task.cont

    def repel(self, task):
        """ Task: perform repel."""
        self.bbase.repel()
        return task.done

    def bump(self, task):
        """ Task: perform bump."""
        mag0 = Vec3(0, 0, 1. / self.bbase.sim_par["size"]) * 10.
        pos = Point3(-1, 0, 0)
        nodes = self.background.descendants()
        bodies = [n.node() for n in nodes if n.type_ is BulletRigidBodyNode]
        for body in bodies:
            mag = mag0 * body.getMass()
            print mag
            body.applyForce(mag, pos)
        #BP()
        return task.done

    def physics_once(self, dt):
        """ Step the physics dt."""
        n_subs = 10
        size_sub = dt / n_subs
        self.bbase.step(dt, n_subs, size_sub)
        # self.bbase.attenuate_velocities(self.bbase.get_bodies())

    def bp(self, task):
        """ Task: break."""
        BP()
        return task.done

    def toggle_debug(self):
        """ Shows/hides debug node."""
        if self.debug_np.isHidden():
            self.debug_np.show()
        else:
            self.debug_np.hide()

    def rotate(self, task):
        """ Task: rotate camera."""
        H = (self.camera_rot.getH() + 1) % 360
        self.camera_rot.setH(H)
        return task.cont

    def rotate90(self, task):
        """ Task: rotate in ticks."""
        angs = [15, 105, 195, 285]
        H = int(self.camera_rot.getH())
        if H in angs:
            self.camera_rot.setH(angs[(angs.index(H) + 1) % len(angs)])
        else:
            self.camera_rot.setH(angs[0])
        return task.done

    def ss_task(self, task):
        """ Task: Take a screenshot."""
        self.screenshot()
        return task.done

    def ssa_task(self, task):
        """ Task: Take a screenshot of every sso."""
        self.screenshot(namePrefix=self.sso.getName() + ".jpg",
                        defaultFilename=False)
        if self.n_ssos - 1 == self.ssos.index(self.sso):
            return task.done
        self.next()
        return task.cont

    def _expunge_events(self):
        """ Turn OFF any non-permanent key handlers."""
        events = self.getAllAccepting()
        for event in set(events).difference(self.permanent_events):
            self.ignore(event)

    def _expunge_tasks(self):
        """ Turn OFF any non-permanent tasks floating around."""
        tasknames = [task.getName() for task in self.taskMgr.getAllTasks()]
        for taskname in set(tasknames).difference(self.permanent_tasks):
            self.taskMgr.remove(taskname)

    def reset_sso(self):
        """ Reset to initial scene state."""
        self.goto_sso(self.ssos.index(self.sso))

    def goto_sso(self, i):
        """ Switches to the i-th SSO."""
        print "SSO %d" % i
        # Remove existing tasks and events.
        self._expunge_tasks()
        self._expunge_events()
        if getattr(self, "sso", False):
            # Detach from physical world.
            self.bbase.remove_all()
            # Reset its state to the initial one.
            self.cache.restore()
            # Detach from scene.
            self.sso.detachNode()
        # Set the new sso.
        self.sso = self.ssos[i]
        self.sso.reparentTo(self.scene)
        self.cache = self.scene.store_tree()
        self.scene.init_tree(tags=("shape",))
        # Attach it to the physics world.
        bnodes = [n for n in self.scene.descendants(type_=PSO)]
        for bnode in bnodes:
            bnode.setCollideMask(BitMask32.allOn())
            bnode.node().setDeactivationEnabled(False)
        self.bbase.attach(bnodes)

    def prev(self):
        """ Task: Go back one SSO."""
        i = max(0, self.ssos.index(self.sso) - 1)
        self.goto_sso(i)

    def next(self):
        """ Task: Go forward one SSO."""
        i = min(self.n_ssos - 1, self.ssos.index(self.sso) + 1)
        self.goto_sso(i)

    def _get_elapsed(self):
        """ Gets the time spent in this phase so far."""
        # Current time.
        current_time = self.taskMgr.globalClock.getFrameTime()
        # Elapsed time in this phase
        elapsed = current_time - self.start_time
        return elapsed

    def run(self):
        # Start with first sso.
        self.goto_sso(0)
        # Call parent's run().
        ShowBase.run(self)

    def exit(self):
        """ Stuff to do before exiting."""
        sys.exit()


def setup_bullet():
    # Initialize physics
    bbase = BulletBase()
    bbase.init()
    # Gravity
    bbase.set_gravity((0., 0., -9.8))
    # Physics step duration
    bbase.sim_par["size"] = 1. / 100
    return bbase


def load(args):
    """ Setup Bullet and load SSOs from input arguments."""
    ssos = []
    for filename in args:
        if path(filename).isfile():
            with open(filename, "r") as fid:
                ssos.append(SSO.load_tree(fid))
        else:
            print "Cannot find file: %s" % filename
    return ssos


if __name__ == "__main__":
    # Command line arguments.
    args = sys.argv[1:]
    # Setup Bullet physics.
    bbase = setup_bullet()
    # Parse the input arguments and load the ssos.
    ssos = load(args)
    # Create the instance.
    app = Viewer()
    app.init_physics(bbase)
    app.init_ssos(ssos)
    # ShowBase's run() starts all the core tasks like graphics etc
    app.run()
