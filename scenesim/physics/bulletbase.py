"""
``scenesim.physics.bulletbase``
===============================

BulletBase interface with Panda3d's Bullet module.

"""
from collections import Iterable
from contextlib import contextmanager
from functools import update_wrapper
from itertools import combinations, izip
from math import isnan, sqrt
from warnings import warn
##
from libpanda import BitMask32, Point3, Quat, Vec3
import numpy as np
from panda3d.bullet import (BulletBaseCharacterControllerNode, BulletBodyNode,
                            BulletConstraint, BulletDebugNode,
                            BulletGenericConstraint, BulletGhostNode,
                            BulletVehicle, BulletWorld)
from panda3d.core import PythonCallbackObject, TransformState
from pandac.PandaModules import NodePath
##
from pdb import set_trace as BP


nan = float("nan")


class BulletBaseError(Exception):
    pass


class DeactivationEnabledWarning(UserWarning):
    pass


class CollisionMonitor(object):
    """ Monitors collisions and records them."""

    def __init__(self, world):
        self.world = world
        self.reset()
        self.use_callback = False
        # self.use_callback = hasattr(self.world, "setContactAddedCallback")
        # # This method does not currently appear to work, so we use the
        # # older method instead.

    def push_notifiers(self, bodies):
        """ Stores initial notification state of bodies, and sets up
        the collisions notification callback."""
        if self.use_callback:
            # Store whether each bodies initially notifies of collisions.
            self.notifies = {body: body.notifiesCollisions()
                             for body in bodies}
            # Set all bodies to notify.
            for body in bodies:
                body.notifyCollisions(True)
            # Add the collision callback.
            self.world.setContactAddedCallback(PythonCallbackObject(self))
        else:
            # This is for Panda3d 1.8, because it doesn't have the
            # contact callback methods.
            # Store the bodies.
            self.bodies = bodies
        # Reset collision count.
        self.reset()

    def pop_notifiers(self):
        """ Restores initial notification state of bodies, and removes
        collisions notification callback."""
        if self.use_callback:
            # Remove the collision callback.
            self.world.clearContactAddedCallback()
            for body, notify in self.notifies.iteritems():
                # Restores initial notification state of bodies.
                body.notifyCollisions(notify)
        else:
            # This is for Panda3d 1.8, because it doesn't have the
            # contact callback methods.
            pass

    def reset(self):
        self._N = 0

    def __nonzero__(self):
        return self._N > 0

    def __iadd__(self, x):
        self._N += x

    def __gt__(self, x):
        return self._N > x

    def __ge__(self, x):
        return self._N >= x

    def __lt__(self, x):
        return self._N < x

    def __le__(self, x):
        return self._N <= x

    def __call__(self, data):
        """ Callback."""
        self.__iadd__(1)

    def detect18(self):
        """ This is for Panda3d 1.8, because it doesn't have the
        contact callback methods."""
        if not self.use_callback:
            def detect(a, b):
                return self.world.contactTestPair(a, b).getNumContacts() > 0
            n = sum(detect(a, b) for a, b in combinations(self.bodies, 2))
            self += n


class JointManager(dict):
    """ Subclass of dict for managing Bullet constraints."""

    def __init__(self, *args, **kwargs):
        # Try adding a bbase instance.
        try:
            self.bbase = kwargs.pop("bbase")
        except KeyError:
            pass
        # Do remaining parent init stuff.
        super(JointManager, self).__init__(*args, **kwargs)

    def __setitem__(self, key, val):
        """ Adds constraint."""
        # First check that the constraint is valid.
        if not isinstance(val, BulletConstraint):
            raise TypeError("Bad type: %s" % type(val))
        # Attach it to self.bbase.
        try:
            # self.bbase.world.attachConstraint(val)
            self.bbase.attach(val)
        except AttributeError:
            BP()
            pass
        # Then do normal dict add.
        super(self.__class__, self).__setitem__(key, val)

    def __delitem__(self, key):
        """ Destroy constraint(s). key must be hashable."""
        try:
            # self.bbase.world.removeConstraint(self[key])
            self.bbase.remove(self[key])
        except AttributeError:
            BP()
            pass
        # Do normal dict delete.
        super(self.__class__, self).__delitem__(key)

    def pop(self, key):
        """ Pop joint from local storage and remove it from Bullet."""
        joint = super(JointManager, self).pop(key)
        try:
            self.bbase.remove(joint)
        except AttributeError:
            pass
        return joint

    @staticmethod
    def make_fixed(np0, np1, type_=BulletGenericConstraint, cfm=0.01, erp=.99):
        """ Make a joint and return it."""
        t0 = np0.getTop()
        t1 = np1.getTop()
        p0 = np0.getPos(t0)
        p1 = np1.getPos(t1)
        q0 = np0.getQuat(t0)
        q1 = np1.getQuat(t1)
        pivot = Point3((p0 + p1) / 2.)
        disp = Point3((p1 - p0) / 2.)
        pivot_np = t0.attachNewNode("pivot-node")
        pivot_np.setPos(pivot)
        s = Vec3(1, 1, 1)
        q = Quat.identQuat()  # pivot_np.getQuat(t0)  #
        #BP()
        # q0 = pivot_np.getQuat(np0)
        # q1 = pivot_np.getQuat(np1)
        q0i = Quat(q0)
        q1i = Quat(q1)
        #BP()
        q0i.invertInPlace()
        q1i.invertInPlace()
        q0i *= q
        q1i *= q
        #BP()
        ts0 = TransformState.makePosQuatScale(disp, q0i, s)
        ts1 = TransformState.makePosQuatScale(-disp, q1i, s)
        pivot_np.removeNode()
        # Create the joint.
        joint = type_(np0.node(), np1.node(), ts0, ts1, False)
        for ax in xrange(4):
            joint.setAngularLimit(ax, 0, 0)
            joint.setLinearLimit(ax, 0, 0)
        joint.setParam(type_.CPErp, erp)
        joint.setParam(type_.CPCfm, cfm)
        joint.setDebugDrawSize(2)
        return joint

    # def attach(self):
    #     """ Attach all of the joints to bbase."""
    #     for joint in self.itervalues():
    #         try:
    #             self.bbase.attach(joint)
    #         except AttributeError:
    #             pass

    # def remove(self):
    #     """ Remove all of the joints from bbase."""
    #     for joint in self.itervalues():
    #         try:
    #             self.bbase.remove(joint)
    #         except AttributeError:
    #             pass


class BulletBase(object):
    """ Manages Panda3d Bullet physics resources and convenience methods."""

    # Bitmasks for each object type. By setting ghost and static
    # objects to different masks, we can filter ghost-to-static
    # collisions.
    ghost_bit = BitMask32.bit(1)
    static_bit = BitMask32.bit(2)
    dynamic_bit = ghost_bit | static_bit
    bw_types = (BulletBaseCharacterControllerNode, BulletBodyNode,
                BulletConstraint, BulletVehicle)

    def __init__(self):
        self.world = None
        # Parameters for simulation.
        self.sim_par = {"size": 1. / 100, "n_subs": 10, "size_sub": 1. / 1000}
        # Initialize axis constraint so that there aren't constraints.
        self.axis_constraint_fac = Vec3(1, 1, 1)
        self.axis_constraint_disp = Vec3(nan, nan, nan)
        # Attribute names of destructable objects.
        self._destructables = ()

    def init(self):
        """ Initialize world and resources. """
        # Initialize world.
        self.world = BulletWorld()

    def destroy(self):
        """ Destroy all resources."""
        for key in self._destructables:
            getattr(self, key).destroy()

    def setup_debug(self):
        debug_node = BulletDebugNode('Debug')
        debug_node.showWireframe(True)
        debug_node.showConstraints(True)
        debug_node.showBoundingBoxes(True)
        debug_node.showNormals(True)
        self.world.setDebugNode(debug_node)
        return debug_node

    @property
    def bodies(self):
        """ Return all bodies (rigid, soft, and ghost) in self.world."""
        bodies = (self.world.getRigidBodies() + self.world.getSoftBodies() +
                  self.world.getGhosts())
        return bodies

    def _constrain_axis(self, body):
        """ Apply existing axis constraints to a body."""
        # Set displacements.
        for axis, (f, d) in enumerate(zip(self.axis_constraint_fac,
                                          self.axis_constraint_disp)):
            if not f and not isnan(d):
                nodep = NodePath(body)
                pos = nodep.getPos()
                pos[axis] = d
                nodep.setPos(pos)
        try:
            # Linear and angular factors of 0 mean forces in the
            # corresponding axis are scaled to 0.
            body.setLinearFactor(self.axis_constraint_fac)
            # Angular factors restrict rotation about an axis, so the
            # following logic selects the appropriate axes to
            # constrain.
            s = sum(self.axis_constraint_fac)
            if s == 3.:
                v = self.axis_constraint_fac
            elif s == 2.:
                v = -self.axis_constraint_fac + 1
            else:
                v = Vec3.zero()
            body.setAngularFactor(v)
        except AttributeError:
            # The body was not a rigid body (it didn't have
            # setLinearFactor method).
            pass

    def set_axis_constraint(self, axis, on, disp=None):
        """ Sets an axis constraint, so that bodies can/cannot
        move in that direction."""
        # Create the constraint vector.
        self.axis_constraint_fac[axis] = int(not on)
        self.axis_constraint_disp[axis] = disp if disp is not None else nan
        # Iterate over bodies, applying the constraint.
        for body in self.bodies:
            self._constrain_axis(body)

    def attach(self, objs, suppress_deact_warn=False):
        """ Attach Bullet objects to the world."""
        if not self.world:
            raise BulletBaseError("No BulletWorld initialized.")
        # Make sure they're iterable.
        if not isinstance(objs, Iterable):
            objs = [objs]
        elif isinstance(objs, dict):
            objs = objs.itervalues()
        bw_objs = []
        for obj in objs:
            if isinstance(obj, NodePath):
                obj = obj.node()
            if isinstance(obj, self.bw_types):
                bw_objs.append(obj)
        # Don't attach ones that are already attached.
        bw_objs = set(bw_objs) - set(self.bodies)
        # Attach them.
        for obj in bw_objs:
            # Warn about deactivation being enabled.
            if (not suppress_deact_warn and
                getattr(obj, "isDeactivationEnabled", lambda: True)()):
                msg = "Deactivation is enabled on object: %s" % obj
                warn(msg, DeactivationEnabledWarning)
            # Apply existing axis constraints to the objects.
            self._constrain_axis(obj)
            # Attach the objects to the world.
            try:
                self.world.attach(obj)
            except AttributeError:
                DeprecationWarning("Upgrade to Panda3d 1.9.")
                for attr in ("attachRigidBody", "attachConstraint",
                             "attachGhost"):
                    attach = getattr(self.world, attr)
                    try:
                        attach(obj)
                    except TypeError:
                        pass
                    else:
                        break

    def remove(self, objs):
        """ Remove Bullet objects to the world."""
        if not self.world:
            raise BulletBaseError("No BulletWorld initialized.")
        # Make sure they're iterable.
        if not isinstance(objs, Iterable):
            objs = [objs]
        elif isinstance(objs, dict):
            objs = objs.itervalues()
        bw_objs = []
        for obj in objs:
            if isinstance(obj, NodePath):
                obj = obj.node()
            if isinstance(obj, self.bw_types):
                bw_objs.append(obj)
        # Remove them.
        for obj in bw_objs:
            # Remove the objects from the world.
            try:
                self.world.remove(obj)
            except AttributeError:
                DeprecationWarning("Upgrade to Panda3d 1.9.")
                for attr in ("removeRigidBody", "removeConstraint",
                             "removeGhost"):
                    remove = getattr(self.world, attr)
                    try:
                        remove(obj)
                    except TypeError:
                        pass
                    else:
                        break

    def remove_all(self):
        """ Remove all objects from world."""
        objs = (self.world.getCharacters() + self.world.getConstraints() +
                self.world.getGhosts() + self.world.getRigidBodies() +
                self.world.getSoftBodies() + self.world.getVehicles())
        self.remove(objs)

    @property
    def gravity(self):
        """ Get gravity on self.world. """
        return self.world.getGravity()

    @gravity.setter
    def gravity(self, gravity):
        """ Set gravity on self.world. """
        self.world.setGravity(Vec3(*gravity))

    def step(self, *args, **kwargs):
        """ Wrapper for BulletWorld.doPhysics."""
        # Defaults.
        dt = args[0] if len(args) > 0 else self.sim_par["size"]
        n_subs = args[1] if len(args) > 1 else self.sim_par["n_subs"]
        size_sub = args[2] if len(args) > 2 else self.sim_par["size_sub"]
        force = kwargs.get("force", None)
        if force:
            bodies, vecpos, dur = force
            dt0 = np.clip(dur, 0., dt)
            n_subs0 = int(np.ceil(n_subs * dt0 / dt))
            dt1 = dt - dt0
            n_subs1 = n_subs - n_subs0 + 1
            for body in bodies:
                body.applyForce(Vec3(*vecpos[0]), Point3(*vecpos[1]))
            # With force.
            self.world.doPhysics(dt0, n_subs0, size_sub)
            for body in bodies:
                body.clearForces()
        else:
            dt1 = dt
            n_subs1 = n_subs
        # With no force.
        self.world.doPhysics(dt1, n_subs1, size_sub)

    @staticmethod
    def attenuate_velocities(bodies, linvelfac=0., angvelfac=0.):
        """ Zeros out the bodies' linear and angular velocities."""
        # Iterate through bodies, re-scaling their velocity vectors
        for body in bodies:
            linvel0 = body.getLinearVelocity()
            angvel0 = body.getAngularVelocity()
            # .normalize() returns True if the length is > 0
            if linvel0.normalize():
                linvel = linvel0 * linvelfac
                body.setLinearVelocity(linvel)
            if angvel0.normalize():
                angvel = angvel0 * angvelfac
                body.setAngularVelocity(angvel)

    def repel(self, n_steps=1000, thresh=10, step_size=0.01):
        """ Performs n_steps physical "repel" steps. """

        @contextmanager
        def repel_context(world):
            """ Sets up a repel context. Gets the bodies, turns off
            gravity, rescales the masses, sets up the collision
            notification callback. """

            def change_contact_thresh(bodies, thresh=0.001):
                """ Adjust the contact processing threshold. This is
                used to make the objects not trigger collisions when
                just barely touching."""
                if isinstance(thresh, Iterable):
                    it = izip(bodies, thresh)
                else:
                    it = ((body, thresh) for body in bodies)
                thresh0 = []
                for body, th in it:
                    thresh0.append(body.getContactProcessingThreshold())
                    body.setContactProcessingThreshold(th)
                return thresh0

            def rescale_masses(bodies):
                """ Rescale the masses so they are proportional to the
                volume."""
                masses, inertias = zip(*[(body.getMass(), body.getInertia())
                                         for body in bodies])
                volumefac = 1.
                for body, mass, inertia in zip(bodies, masses, inertias):
                    # Compute the mass-normalized diagonal elements of the
                    # inertia tensor.
                    if mass > 0.:
                        it = inertia / mass * 12
                        # Calculate volume from the mass-normalized
                        # inertia tensor (from wikipedia).
                        h = sqrt((it[0] - it[1] + it[2]) / 2)
                        w = sqrt(it[2] - h ** 2)
                        d = sqrt(it[1] - w ** 2)
                        volume = h * w * d
                        # Change the mass.
                        body.setMass(volume * volumefac)
                return masses

            # Get the bodies.
            bodies = world.getRigidBodies()
            # Turn gravity off.
            gravity = world.getGravity()
            world.setGravity(Vec3.zero())
            # Tighten the contact processing threshold slightly.
            delta = -0.001
            cp_thresh = change_contact_thresh(bodies, thresh=delta)
            # Adjust masses.
            masses = rescale_masses(bodies)
            # Adjust sleep thresholds.
            deactivations = [b.isDeactivationEnabled() for b in bodies]
            for body in bodies:
                body.setDeactivationEnabled(False)
            # Zero out velocities.
            self.attenuate_velocities(bodies)
            # Collisions monitor.
            collisions = CollisionMonitor(world)
            collisions.push_notifiers(bodies)
            ## Finish __enter__.
            yield bodies, collisions
            ## Start __exit__.
            collisions.pop_notifiers()
            # Zero out velocities.
            self.attenuate_velocities(bodies)
            # Restore the contact processing threshold.
            change_contact_thresh(bodies, thresh=cp_thresh)
            # Set masses back.
            for body, mass in zip(bodies, masses):
                body.setMass(mass)
                # Turn gravity back on.
                world.setGravity(gravity)
            for body, d in zip(bodies, deactivations):
                body.setDeactivationEnabled(d)

        # Operate in a context that changes the masses, turns off
        # gravity, adds collision monitoring callback, etc.
        with repel_context(self.world) as (bodies, collisions):
            # Loop through the repel simulation.
            done_count = 0
            for istep in xrange(n_steps):
                # Take one step.
                self.world.doPhysics(step_size, 1, step_size)
                # HACK: The following can be removed once Panda3d 1.9
                # is installed (and the method can be removed from
                # CollisionMonitor).
                collisions.detect18()
                # Increment done_count, only if there are no contacts.
                if collisions:
                    done_count = 0
                else:
                    done_count += 1
                if any(body.getMass() > 0. and not body.isActive()
                       for body in bodies):
                    BP()
                # Stop criterion.
                if done_count >= thresh:
                    break
                # Zero-out/re-scale velocities.
                linvelfac = bool(collisions) * 0.001
                angvelfac = bool(collisions) * 0.001
                self.attenuate_velocities(bodies, linvelfac, angvelfac)
                # Reset collisions.
                collisions.reset()
        return istep

    @classmethod
    def add_collide_mask(cls, func0):
        """ Decorator. Initializes ghost, static, and dynamic nodes
        with the appropriate collide masks."""
        def func(*args, **kwargs):
            # Create node using func0.
            node = func0(*args, **kwargs)
            # Determine collide mask.
            if isinstance(node, BulletGhostNode):
                bit = cls.ghost_bit
            elif node.getMass() == 0.:
                bit = cls.static_bit
            else:
                bit = cls.dynamic_bit
            # Set collide mask.
            node.setCollideMask(bit)
            return node
        return update_wrapper(func0, func)

    @staticmethod
    def add_ghostnode(node):
        """ Adds a child ghostnode to a node as a workaround for the
        ghost-static node collision detection problem."""
        name = "%s-ghost" % node.getName()
        ghost = NodePath(BulletGhostNode(name))
        ghost.reparentTo(node)
        return ghost
