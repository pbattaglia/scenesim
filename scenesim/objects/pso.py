"""
``scenesim.objects.pso``
========================

Physics SceneSim objects.

"""
from abc import ABCMeta, abstractmethod
from collections import Iterable, OrderedDict
from contextlib import contextmanager
from ctypes import c_float
from functools import wraps
from itertools import izip
import re
##
import numpy as np
import panda3d.bullet as p3b
from panda3d.bullet import BulletBodyNode, BulletGhostNode, BulletRigidBodyNode
from panda3d.core import (BitMask32, Mat4, NodePathCollection, Point3,
                          TransformState, Vec3)
##
from scenesim.objects.sso import SSO
##
from pdb import set_trace as BP


def cast_c_float(func):
    """ Decorator for casting a function's return value to ctypes.f32."""
    def func_c_float(*args, **kwargs):
        return c_float(func(*args, **kwargs)).value
    return func_c_float


class BaseShape(list):
    """ Base class for shape objects."""
    __metaclass__ = ABCMeta
    _type_rx = re.compile("Bullet(.+)Shape")

    def __new__(cls, *args, **kwargs):
        """ Set cls._bshape by reading from bases of derived class."""
        obj = super(BaseShape, cls).__new__(cls)
        cls.name = cls.read_name(cls.bshape)
        return obj

    @classmethod
    def _fix_args(cls, args):
        """ Standardize the BulletShape arguments."""
        args1 = cls.args0.copy()
        if isinstance(args, dict):
            args1.update(args)
        else:
            for key, prm in izip(args1, args):
                args1[key] = prm
        return args1.values()

    @staticmethod
    def _fix_xform(T):
        """ Converts T into a valid xform. Returns None on fail."""
        # If T has "flat" attribute, it is an ndarray and that should
        # be returned. Otherwise just return T.
        if not T:
            xform = TransformState.makeIdentity()
        else:
            if not isinstance(T, TransformState):
                mat = Mat4(*T.flat) if hasattr(T, "flat") else Mat4(T)
                xform = TransformState.makeMat(mat)
            else:
                xform = T
        return xform

    @classmethod
    def _fix_prms(cls, p):
        """ Standardizes parameters."""
        if not isinstance(p, Iterable) or isinstance(p, str) or len(p) > 2:
            raise ValueError("Bad input: %s" % p)
        elif len(p) == 0:
            prms = [(), None]
        elif len(p) == 1:
            prms = [p[0], None]
        else:
            prms = list(p)
        # Make prms into list with fixed args and xform.
        prms = [cls._fix_args(prms[0]), cls._fix_xform(prms[1])]
        return prms

    def __init__(self, *prms):
        """ Initialize."""
        prms = self._fix_prms(prms)
        super(BaseShape, self).__init__(prms)

    def init(self):
        """ Return the initialized Bullet*Shape and the xform."""
        args, xform = self
        bshape = self.bshape(*args)
        return bshape, xform

    @classmethod
    def read_name(cls, bshape):
        return cls._type_rx.match(bshape.__name__).group(1)

    @classmethod
    def read_params(cls, bshape):
        return [getattr(bshape, "get%s" % key)() for key in cls.args0]

    @abstractmethod
    def scale(self, scale):
        """ Scales shape arguments."""
        pass

    def shift(self, pos=(0, 0, 0), quat=(1, 0, 0, 0)):
        """ Translate and rotate shape's transform."""
        ones = Vec3(1, 1, 1)
        T = TransformState.makePosQuatScale(pos, quat, ones)
        xform = T.compose(self[1])
        self[1] = xform

    def transform(self, node, other=None):
        """ Shift and scale the shape by node's transform."""
        if other is None:
            other = node.getParent()
        scale = node.get_scale(other)
        pos = node.get_pos(other)
        quat = node.get_quat(other)
        self.scale(scale)
        self.shift(pos=pos, quat=quat)


class BoxShape(BaseShape):
    """ BulletBoxShape."""
    bshape = p3b.BulletBoxShape
    args0 = OrderedDict((("HalfExtentsWithMargin", Vec3(0.5, 0.5, 0.5)),))

    def scale(self, scale):
        args = (Vec3(*[s * a for s, a in izip(scale, self[0][0])]),)
        self[0] = args


class CapsuleShape(BaseShape):
    """ BulletCapsuleShape."""
    bshape = p3b.BulletCapsuleShape
    args0 = OrderedDict((("Radius", 0.5), ("HalfHeight", 0.5)))

    def scale(self, scale):
        if scale[0] != scale[1]:
            raise ValueError("%s does not support anisotropic x,y scaling" %
                             self.bshape)
        self[0] = (scale[0] * self[0][0], scale[2] * self[0][1])


class ConeShape(BaseShape):
    """ BulletConeShape."""
    bshape = p3b.BulletConeShape
    args0 = OrderedDict((("Radius", 0.5), ("Height", 1.)))

    def scale(self, scale):
        if scale[0] != scale[1]:
            raise ValueError("%s does not support anisotropic x,y scaling" %
                             self.bshape)
        self[0] = (scale[0] * self[0][0], scale[2] * self[0][1])


class ConvexHullShape(BaseShape):
    """ BulletConvexHullShape."""
    bshape = p3b.BulletConvexHullShape
    args0 = OrderedDict()

    def __init__(self, *args, **kwargs):
        super(ConvexHullShape, self).__init__()
        self.array = args[0][0] if args[0] else None

    def init(self):
        """ Return the initialized Bullet*Shape and the xform."""
        bshape, xform = super(ConvexHullShape, self).init()
        if self.array is not None:
            array = map(lambda a: Vec3(*a), self.array)
            bshape.add_array(array)
        return bshape, xform

    def scale(self, scale):
        if scale[0] != scale[1]:
            raise ValueError("%s does not support anisotropic x,y scaling" %
                             self.bshape)
        self[0] = (scale[0] * self[0][0], scale[2] * self[0][1])


class CylinderShape(BaseShape):
    """ BulletCylinderShape. Height is along the Z axis."""
    bshape = p3b.BulletCylinderShape
    args0 = OrderedDict((("Radius", 0.5), ("Height", 1.)))

    def scale(self, scale):
        if scale[0] != scale[1]:
            raise ValueError("%s does not support anisotropic x,y scaling" %
                             self.bshape)
        self[0] = (scale[0] * self[0][0], scale[2] * self[0][1])


class PlaneShape(BaseShape):
    """ BulletPlaneShape."""
    bshape = p3b.BulletPlaneShape
    args0 = OrderedDict((("PlaneNormal", Vec3(0, 0, 1)), ("PlaneConstant", 0)))

    def scale(self, scale):
        s = self[0][0].dot(Vec3(*scale)) / self[0][0].length()
        args = (self[0][0], self[0][1] * s)
        self[0] = args


class SphereShape(BaseShape):
    """ BulletSphereShape."""
    bshape = p3b.BulletSphereShape
    args0 = OrderedDict((("Radius", 0.5),))

    def scale(self, scale):
        if (scale[0] != scale[1]) or (scale[0] != scale[2]):
            raise ValueError("%s does not support anisotropic x,y,z scaling" %
                             self.bshape)
        self[0] = (scale[0] * self[0][0],)


class ShapeManager(object):
    """ Utility class for making *Shape objects and reading BulletShapes."""

    _name2shape = {
        "Box": BoxShape,
        "Capsule": CapsuleShape,
        "Cone": ConeShape,
        "ConvexHull": ConvexHullShape,
        "Cylinder": CylinderShape,
        "Plane": PlaneShape,
        "Sphere": SphereShape}

    @classmethod
    def make1(cls, val):
        """ Initialize a *Shape object from input."""
        if isinstance(val, str):
            type_ = cls._name2shape[val]
            prms = [(), None]
        else:
            type_ = cls._name2shape[val[0]]
            prms = val[1:]
        return type_(*prms)

    @classmethod
    def make(cls, vals):
        """ Standardizes list of vals."""
        if isinstance(vals, str):
            vals = [[vals, (), None]]
        elif isinstance(vals[0], str):
            vals = [vals]
        # Verify that vals contains valid names and xform matrices.
        shapes = [cls.make1(val) for val in vals]
        return shapes

    @classmethod
    def read(cls, node):
        """ Get shape list from Bullet*Shape(s)."""
        # Get valid node.
        try:
            node = node.node()
        except AttributeError:
            pass
        # Get shapes.
        n_shapes = node.getNumShapes()
        if n_shapes == 0:
            # For no shape, return "".
            shapes = []
        else:
            # For 1+ shapes.
            parent = node.getParent()
            node.detachNode()
            shapes = []
            for i in xrange(n_shapes):
                # Get shape and shape's matrix.
                bshape = node.getShape(i)
                # Get name.
                name = cls.read_name(bshape)
                # Get params.
                Shape = cls._name2shape[name]
                params = Shape.read_params(bshape)
                # Get xform.
                xform = TransformState.makeMat(node.getShapeMat(i))
                # Store.
                shapes.append((name, params, xform))
            node.reparentTo(parent)
        return shapes


class ShapeList(list):
    """ List of *Shapes."""

    def __init__(self, val=""):
        super(ShapeList, self).__init__(ShapeManager.make(val))

    def __setitem__(self, key, val):
        super(ShapeList, self).__setitem__(key, ShapeManager.make1(val))

    def __add__(self, val):
        super(ShapeList, self).__add__(ShapeManager.make(val))

    def __iadd__(self, val):
        super(ShapeList, self).__iadd__(ShapeManager.make(val))

    def append(self, val):
        super(ShapeList, self).append(ShapeManager.make1(val))

    def extend(self, val):
        super(ShapeList, self).extend(ShapeManager.make(val))

    def insert(self, key, val):
        super(ShapeList, self).insert(key, ShapeManager.make1(val))

    def init(self):
        """ Build the BulletShapes (bshapes) and TransformStates
        (xforms)."""
        bshapes = []
        xforms = []
        for shape in self:
            bshape, xform = shape.init()
            bshapes.append(bshape)
            xforms.append(xform)
        return bshapes, xforms


class PSO(SSO):
    """ Bullet physics object."""

    type_ = BulletBodyNode
    _prop_tags = ("friction", "restitution", "shape", "deactivation_enabled")
    _res_tags = ("shape",)

    def __init__(self, *args, **kwargs):
        # Converts args so they're appropriate for self.type_.
        if len(args) == 0:
            args = ("",)
        if isinstance(args[0], (str, unicode)):
            args = (self.type_(args[0]),) + args[1:]
            tag = self.__class__
        else:
            tag = None
        ## Using super fails, probably because NodePath is a C++ class.
        # super(PSO, self).__init__(self, *new_args, **kwargs)
        SSO.__init__(self, *args, **kwargs)
        if tag:
            self.setPythonTag("sso", tag)

    @wraps(type_.set_friction, assigned=("__name__", "__doc__"))
    def set_friction(self, friction):
        self.node().set_friction(friction)

    @cast_c_float
    @wraps(type_.get_friction, assigned=("__name__", "__doc__"))
    def get_friction(self):
        return self.node().get_friction()

    @wraps(type_.set_restitution, assigned=("__name__", "__doc__"))
    def set_restitution(self, restitution):
        self.node().set_restitution(restitution)

    @cast_c_float
    @wraps(type_.get_restitution, assigned=("__name__", "__doc__"))
    def get_restitution(self):
        return self.node().get_restitution()

    def set_shape(self, shape):
        self.setPythonTag("shape", shape)

    def get_shape(self):
        if "shape" not in self.getPythonTagKeys():
            self.set_shape("")
        return self.getPythonTag("shape")

    def add_shape(self, shapes):
        """ Adds the shape."""
        # Construct the BulletShapes.
        bshapes, xforms = shapes.init()
        for bshape, xform in izip(bshapes, xforms):
            # Add each to the BulletBodyNode.
            self.node().addShape(bshape, xform)

    def create_shape(self):
        """ Initializes a BulletShape(s)."""
        shapes = ShapeList(self.get_shape())
        self.add_shape(shapes)
        self.setTag("resource", "shape")

    def delete_shape(self):
        """ Destroys the BulletShape(s)."""
        shapes = self.node().getShapes()
        for shape in shapes:
            self.node().removeShape(shape)
        self.clearTag("resource")

    @wraps(type_.set_deactivation_enabled, assigned=("__name__", "__doc__"))
    def set_deactivation_enabled(self, is_enabled):
        return self.node().set_deactivation_enabled(is_enabled)

    @wraps(type_.is_deactivation_enabled, assigned=("__name__", "__doc__"))
    def get_deactivation_enabled(self):
        return self.node().is_deactivation_enabled()


class RBSO(PSO):
    type_ = BulletRigidBodyNode
    _prop_tags = ("linear_velocity", "angular_velocity", "mass", "gravity",
                  "into_collide_mask")
    _res_tags = ()

    @wraps(type_.set_mass, assigned=("__name__", "__doc__"))
    def set_mass(self, mass):
        self.node().set_mass(mass)

    @cast_c_float
    @wraps(type_.get_mass, assigned=("__name__", "__doc__"))
    def get_mass(self):
        return self.node().get_mass()

    @wraps(type_.set_linear_velocity, assigned=("__name__", "__doc__"))
    def set_linear_velocity(self, linear_velocity):
        self.node().set_linear_velocity(linear_velocity)

    @wraps(type_.get_linear_velocity, assigned=("__name__", "__doc__"))
    def get_linear_velocity(self):
        return self.node().get_linear_velocity()

    @wraps(type_.set_angular_velocity, assigned=("__name__", "__doc__"))
    def set_angular_velocity(self, angular_velocity):
        self.node().set_angular_velocity(angular_velocity)

    @wraps(type_.get_angular_velocity, assigned=("__name__", "__doc__"))
    def get_angular_velocity(self):
        return self.node().get_angular_velocity()

    @wraps(type_.set_gravity, assigned=("__name__", "__doc__"))
    def set_gravity(self, grav):
        return self.node().set_gravity(grav)

    @wraps(type_.get_gravity, assigned=("__name__", "__doc__"))
    def get_gravity(self):
        return self.node().get_gravity()

    @wraps(type_.set_into_collide_mask, assigned=("__name__", "__doc__"))
    def set_into_collide_mask(self, into_collide_mask):
        if isinstance(into_collide_mask, int):
            into_collide_mask = BitMask32.bit(into_collide_mask)
        self.node().set_into_collide_mask(into_collide_mask)

    @wraps(type_.get_into_collide_mask, assigned=("__name__", "__doc__"))
    def get_into_collide_mask(self):
        return self.node().get_into_collide_mask()

    def set_center_of_mass(self, com, other=None):
        """Sets center of mass of object.

        Args:
            pos (seq): Position coordinates (3 elements).

        Kwargs:
            other (NodePath): The node which the new state is set relative to.
                              (default=self.getParent())
        """
        if other is None:
            other = self.getParent()
        pos = self.get_pos(other)
        ts_com = TransformState.makePos(-Vec3(com))
        for i in xrange(self.node().getNumShapes()):
            # Compute the new transform.
            mat = self.node().getShapeMat(i)
            ts0 = TransformState.makeMat(mat)
            ts = ts0.compose(ts_com)
            # Change the transform.
            shape = self.node().getShape(i)
            self.node().removeShape(shape)
            self.node().addShape(shape, ts)
        # Change the object's shape
        self.set_pos(other, com + pos)

    # def get_center_of_mass(self):
    #     """Return center of mass of object, with respect to node's
    #     coordinate frame.

    #     Return:
    #         (seq): Position coordinates (3 elements).
    #     """
    #     pos = self.node().get
    #     return pos


class CPSO(RBSO):
    """ Bullet physics object, specialized for compound shapes."""

    def __init__(self, *args, **kwargs):
        if len(args) == 0:
            args = ("compound",)
        super(CPSO, self).__init__(*args, **kwargs)

    @property
    def components(self):
        return self.descendants(depths=[1], type_=PSO)

    @contextmanager
    def _preserve_child_tranforms(self):
        """ Remember transforms of existing children to avoid
        center-of-mass shift."""
        parent = self.getParent()
        descendants = self.descendants(depths=[1])
        # Remember child transforms.
        mats = [child.get_mat(parent) for child in descendants]
        yield parent
        # Update transforms of existing children.
        for descendant, mat in izip(descendants, mats):
            descendant.set_mat(parent, mat)

    def _compute_shapes(self):
        """ Computes shapes from self.components."""
        # Compute mass and center-of-mass.
        masses = []
        poses = []
        psos = self.descendants(depths=[1], type_=PSO)
        parent = self.getParent()
        for pso in psos:
            mass = pso.get_mass()
            pos = pso.get_pos(parent)
            if mass == 0.:
                com = pos
                break
            poses.append(pos)
            masses.append(mass)
        else:
            mass = np.sum(masses)
            com = Point3(*(np.sum(np.array(poses).T * masses, axis=-1) / mass))
        self.set_mass(mass)
        with self._preserve_child_tranforms() as parent:
            self.set_pos(parent, com)
        # Add shapes from PSOs.
        vals = []
        for pso in psos:
            shapes0 = ShapeList(pso.get_shape())
            for shape0 in shapes0:
                name = shape0.name
                args0, xform0 = shape0
                if name != "Box":
                    print("Can't handle that shape: %s" % name)
                    BP()
                shape = ShapeManager.make1((name, args0, xform0))
                shape.transform(pso, other=self)
                # scale = pso.get_scale(self)
                # pos = pso.get_pos(self)
                # quat = pso.get_quat(self)
                # shape.scale(scale)
                # shape.shift(pos, quat)
                val = (name, shape[0], shape[1])
                vals.append(val)
        # Set compound object's shapes tag.
        self.set_shape(vals)

    def add(self, psos):
        """ Add sequence of PSOs to compound object."""
        NodePathCollection(psos).wrtReparentTo(self)
        self._compute_shapes()

    def remove(self, psos):
        """ Remove sequence of PSOs from compound object."""
        with self._preserve_child_tranforms():
            NodePathCollection(psos).detach()
        if self.getNumChildren() > 0:
            self._compute_shapes()

    def destroy_component_shapes(self):
        """ Destroys the shape resources of the component PSOs."""
        for pso in self.components:
            pso.destroy_resources(tags=("shape",))

    def remove_component_bodies(self, bbase):
        """ Destroys the shape resources of the component PSOs."""
        bbase.remove(self.components)

    def init_tree(self, tags=None):
        """ Overrides parent's init_tree() so that components' shapes are not
        initialized."""
        super(CPSO, self).init_tree(tags=tags)
        # if tags is None or "shape" in tags:
        self.destroy_component_shapes()


class GHSO(PSO):
    """ PSO subclass for `BulletGhostNode`s."""

    type_ = BulletGhostNode
    _prop_tags = ("num_overlapping_nodes", "overlapping_nodes")
    _res_tags = ()

    @wraps(type_.get_num_overlapping_nodes, assigned=("__name__", "__doc__"))
    def get_num_overlapping_nodes(self):
        return self.node().get_num_overlapping_nodes()

    @wraps(type_.get_overlapping_nodes, assigned=("__name__", "__doc__"))
    def get_overlapping_nodes(self):
        return self.node().get_overlapping_nodes()


## TODO
#



# class A(object):
#     def __init__(self):
#         print "A.__init__()"
# class AA(A):
#     def __init__(self):
#         print "AA.__init__()"
# class B(AA):
#     def __init__(self):
#         print "B.__init__()"
#         super(B, self).__init__()
# class C(AA):
#     def __init__(self):
#         print "C.__init__()"
#         super(C, self).__init__()
# class D(C, B):
#     pass
# d = D()
# print D.mro()
