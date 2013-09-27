""" Physics objects."""
from collections import Iterable, OrderedDict
from contextlib import contextmanager
from ctypes import c_float
from functools import wraps
from itertools import izip, dropwhile
import re
##
from libpanda import Mat4, Point3, TransformState, Vec3
import numpy as np
import panda3d.bullet as p3b
from panda3d.bullet import BulletBodyNode, BulletGhostNode, BulletRigidBodyNode
from panda3d.core import NodePathCollection
##
from scenesim.objects.sso import SSO


def cast_c_float(func):
    """ Decorator for casting a function's return value to ctypes.f32."""
    def func_c_float(*args, **kwargs):
        return c_float(func(*args, **kwargs)).value
    return func_c_float


# class AbstractFamily(object):
#     """ Family-specific builders derive from this."""

#     def __init__(self, family):
#         self.family = family

#     def get_name(self):
#         return self.name

#     def make_shape(self):
#         raise NotImplementedError("make_shape() not implemented")


# class FactoryManager(object):
#     """ Manage builders by family."""

#     def __init__(self, current_family=None):
#         self.builders = {}
#         self.family = current_family

#     def set_family(self, family):
#         if not family:
#             raise ValueError("Empty family")
#         self.family = family

#     def add(self, builder):
#         name = builder.get_name()
#         self.builders[name] = builder

#     def make_shape(self):
#         self._check_state()
#         return self.builders[self.family].make_shape()

#     def check_state(self):
#         if not self.family:
#             raise ValueError("No family specified.")
#         if self.family not in self.builders:
#             raise ValueError("Unknown family: %s" % self.family)


class BaseShape(object):
    """ Base class for shape objects."""

    _type_rx = re.compile("Bullet(.+)Shape")

    def __new__(cls, *args):
        """ Set cls._bshape by reading from bases of derived class."""
        obj = super(BaseShape, cls).__new__(cls)
        cls.bshape = cls.__bases__[1]
        cls.name = cls._type_rx.match(cls.bshape.__name__).group(1)
        return obj

    def _fix_args(self, args):
        """ Standardize the BulletShape arguments."""
        args1 = self.args0.copy()
        if isinstance(args, dict):
            args1.update(args)
        else:
            for key, prm in izip(args1, args):
                args1[key] = prm
        return args1

    def __init__(self, *args):
        """ Initialize."""
        n_args = len(self.args0)
        if len(args) > n_args:
            raise ValueError("__init__(*args) takes 0 or 1 arguments.")
        elif len(args) == 0:
            args = OrderedDict()
        elif isinstance(args[0], dict):
            args = args[0]
        args1 = self._fix_args(args)
        super(BaseShape, self).__init__(*args1.values())

    @classmethod
    def _fix_xform(cls, T):
        """ Converts T into a valid xform. Returns None on fail."""
        # If T has "flat" attribute, it is an ndarray and that should
        # be returned. Otherwise just return T.
        if not T:
            xform = TransformState.makeIdentity()
        else:
            if not isinstance(T, TransformState):
                try:
                    mat = Mat4(*T.flat) if hasattr(T, "flat") else Mat4(T)
                except TypeError:
                    mat = None
                xform = TransformState.makeMat(mat)
            else:
                xform = T
        return xform

    @classmethod
    def fix_prms(cls, prms):
        """ Standardizes parameters."""
        if prms == "":
            prms = ("",)
        elif isinstance(prms, str):
            # prms is a string. Put defaults in params and None in xform
            # (which will be made the identity transform).
            prms = (prms, (), None)
        elif isinstance(prms, Iterable) and len(prms) != 3:
            raise ValueError("Bad input.")
        else:
            # Make prms into tuple S.
            prms = tuple(prms)
        # Check that the shape is valid.
        if prms[0] not in ("", cls.name):
            raise ValueError("Bad shape: %s" % prms[0])
        if prms[0]:
            # Fix xform.
            xform = cls._fix_xform(prms[2])
            if xform is None:
                raise ValueError("Bad xform: %s" % str(prms[2]))
            prms = (prms[0], prms[1], xform)
        else:
            prms = None
        return prms

    @classmethod
    def init(cls, prms):
        prms = cls.fix_prms(prms)






class BaseShape(tuple):
    """ Base class for shape objects."""

    _type_rx = re.compile("(.+)Shape")

    def __new__(cls):
        """ Set cls._bshape by reading from bases of derived class."""
        obj = super(BaseShape, cls).__new__(cls)
        cls.name = cls._type_rx.match(cls.__name__).group(1)
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
    def fix_prms(cls, prms0):
        """ Standardizes parameters."""
        if isinstance(prms0, str):
            # prms is a string. Put defaults in params and None in xform
            # (which will be made the identity transform).
            name = prms0
            prms = ((), None)
        elif isinstance(prms0, Iterable) and len(prms0) == 1:
            name = prms0[0]
            prms = ((), None)
        elif isinstance(prms0, Iterable) and len(prms0) == 2:
            name = prms0[0]
            prms = (prms0[1], None)
        elif not (isinstance(prms0, Iterable) and len(prms0) == 3):
            raise ValueError("Bad input.")
        # Check that the shape is valid.
        if name is not cls.name:
            raise ValueError("Bad shape name: %s" % name)
        # Make prms into tuple with fixed args and xform.
        prms = (cls._fix_args(prms[0]), cls._fix_xform(prms[1]))
        return prms

    def __init__(self, *prms):
        """ Initialize."""
        if not prms or not prms[0]:
            prms = self.name
        else:
            prms = prms[0]
        prms = self.fix_prms(prms)
        super(BaseShape, self).__init__(prms)

    def init(self):
        """ Return the initialized Bullet*Shape and the xform."""
        args, xform = self
        bshape = self.bshape(*args)
        return bshape, xform

    @classmethod
    def get_name(cls, bshape):
        return cls._type_rx.match(bshape.__name__).group(1)

    @classmethod
    def get_params(cls, bshape, name=None):
        if name is None:
            name = cls.get_name(bshape)
        return tuple([getattr(bshape, "get%s" % key)() for key in cls.args0])
    

class BoxShape(BaseShape):
    """ BulletBoxShape."""
    bshape = p3b.BulletBoxShape
    args0 = OrderedDict((("HalfExtentsWithMargin", Vec3(0.5, 0.5, 0.5)),))


class CapsuleShape(BaseShape):
    """ BulletCapsuleShape."""
    bshape = p3b.BulletCapsuleShape
    args0 = OrderedDict((("Radius", 0.5), ("HalfHeight", 0.5)))


class ConeShape(BaseShape):
    """ BulletConeShape."""
    bshape = p3b.BulletConeShape
    args0 = OrderedDict((("Radius", 0.5), ("Height", 1.)))


class CylinderShape(BaseShape):
    """ BulletCylinderShape."""
    bshape = p3b.BulletCylinderShape
    args0 = OrderedDict((("Radius", 0.5), ("Height", 1.)))


class PlaneShape(BaseShape):
    """ BulletPlaneShape."""
    bshape = p3b.BulletPlaneShape
    args0 = OrderedDict((("PlaneNormal", Vec3(0, 0, 1)), ("PlaneConstant", 0)))


class SphereShape(BaseShape):
    """ BulletSphereShape."""
    bshape = p3b.BulletSphereShape
    args0 = OrderedDict((("Radius", 0.5),))


class ShapeManager(list):
    """ Subclass of list for storing *Shapes."""

    _name2shape = {
        "Box": BoxShape,
        "Capsule": CapsuleShape,
        "Cone": ConeShape,
        "Cylinder": CylinderShape,
        "Plane": PlaneShape,
        "Sphere": SphereShape}

    @classmethod
    def _init1(cls, val):
        """ Initialize a *Shape object from input."""
        if isinstance(val, str):
            type_ = cls._name2shape[val]
        else:
            type_ = cls._name2shape[val[0]]
        shape = type_(val)

    @classmethod
    def _init(cls, vals):
        """ Standardizes list of vals."""
        if isinstance(vals, str):
            raise TypeError("`vals` is str, must be Iterable: %s" % vals)
        # Verify that vals contains valid names and xform matrices.
        shapes = [cls._init1(val) for val in vals]
        return shapes
        
    def __init__(self, val=""):
        super(ShapeManager, self).__init__(self._init(val))

    def __setitem__(self, key, val):
        super(ShapeManager, self).__setitem__(key, self._init1(val))

    def __add__(self, val):
        super(ShapeManager, self).__add__(self._init(val))

    def __iadd__(self, val):
        super(ShapeManager, self).__iadd__(self._init(val))

    def append(self, val):
        super(ShapeManager, self).append(self._init1(val))

    def extend(self, val):
        super(ShapeManager, self).extend(self._init(val))

    def insert(self, key, val):
        super(ShapeManager, self).insert(key, self._init1(val))

    @classmethod
    def read(cls, node):
        """ Gets 'shape' from Bullet*Shape(s)."""
        # Get valid node.
        try:
            node = node.node()
        except AttributeError:
            pass
        # Get shapes.
        n_shapes = node.getNumShapes()
        if n_shapes == 0:
            # For no shape, return "".
            shape = cls("")
        else:
            # For 1+ shapes.
            parent = node.getParent()
            node.detachNode()
            vals = []
            for i in xrange(n_shapes):
                # Get shape and shape's matrix.
                bshape = node.getShape(i)
                # Get name.
                name = BaseShape.get_name(bshape)
                # Get params.
                params = BaseShape.get_params(bshape, name=name)
                # Get xform.
                xform = TransformState.makeMat(node.getShapeMat(i))
                # Store.
                vals.append((name, params, xform))
            node.reparentTo(parent)
            # Create *Shape object.
            shape = cls(vals)
        return shape

    def init(self):
        """ Build the BulletShapes (shapes) and TransformStates
        (xforms). We need to apply the node's scale to the parameters
        of the BulletShape, and also before translating the shapes'
        xforms."""
        shapes = []
        xforms = []
        for name, params, xform in self:
            shape = self._init(name, params)
            shapes.append(shape)
            xforms.append(xform)
        return shapes, xforms


# class BShapeManager(list):
#     """ Subclass of list for storing shapes used for creating
#     BulletShapes."""

#     name2shape = {
#         "Box": p3b.BulletBoxShape,
#         "Capsule": p3b.BulletCapsuleShape,
#         "Cone": p3b.BulletConeShape,
#         "Cylinder": p3b.BulletCylinderShape,
#         "Plane": p3b.BulletPlaneShape,
#         "Sphere": p3b.BulletSphereShape}
#     shape2name = {v: k for k, v in name2shape.iteritems()}
#     parameters = {
#         "Box": OrderedDict((("HalfExtentsWithMargin", Vec3(0.5, 0.5, 0.5)),)),
#         "Capsule": OrderedDict((("Radius", 0.5), ("HalfHeight", 0.5))),
#         "Cone": OrderedDict((("Radius", 0.5), ("Height", 1.))),
#         "Cylinder": OrderedDict((("Radius", 0.5), ("Height", 1.))),
#         "Plane": OrderedDict((("PlaneNormal", Vec3(0, 0, 1)),
#                               ("PlaneConstant", 0.))),
#         "Sphere": OrderedDict((("Radius", 0.5),))}

#     def __init__(self, val=""):
#         super(self.__class__, self).__init__(self._safe_set(val))

#     def __setitem__(self, key, val):
#         super(self.__class__, self).__setitem__(key, self._safe_set1(val))

#     def __add__(self, val):
#         super(self.__class__, self).__add__(self._safe_set(val))

#     def __iadd__(self, val):
#         super(self.__class__, self).__iadd__(self._safe_set(val))

#     def append(self, val):
#         super(self.__class__, self).append(self._safe_set1(val))

#     def extend(self, val):
#         super(self.__class__, self).extend(self._safe_set(val))

#     def insert(self, key, val):
#         super(self.__class__, self).insert(key, self._safe_set1(val))

#     @classmethod
#     def _safe_set1(cls, X):
#         """ Standardizes 1-element X."""
#         if X == "":
#             S = ("",)
#         elif isinstance(X, str):
#             # X is a string. Put defaults in params and None in xform
#             # (which will be made the identity transform).
#             S = (X, (), None)
#         elif isinstance(X, Iterable) and len(X) != 3:
#             raise ValueError("Bad input.")
#         else:
#             # Make X into tuple S.
#             S = tuple(X)
#         # Check that the shape is valid.
#         if S[0] not in ("",) + tuple(cls.parameters):
#             raise ValueError("Bad shape: %s" % S[0])
#         if S[0]:
#             # Make xform a TransformState.
#             xform = cls._xform(S[2])
#             if xform is None:
#                 raise ValueError("Bad xform: %s" % str(S[2]))
#             shape = (S[0], S[1], xform)
#         else:
#             shape = None
#         return shape

#     def _safe_set(self, X):
#         """ Standardizes list of Xs."""
#         # Organize into list of tuples, S: [(str, param, TransformState), ...].
#         if X == "":
#             X = []
#         elif isinstance(X, str):
#             X = [X]
#         elif (isinstance(X, Iterable) and len(X) == 3 and
#               isinstance(X[0], str) and self._xform(X[2])):
#             X = [X]
#         # Verify that X contains valid names and xform matrices.
#         shape = []
#         for x in X:
#             # Standardize the shape.
#             s = self._safe_set1(x)
#             # If invalid, fail.
#             if not s:
#                 raise ValueError("Bad shape: %s" % str(x))
#             shape.append(s)
#         return shape

#     @classmethod
#     def _xform(cls, T):
#         """ Converts T into a valid xform. Returns None on fail."""
#         # If T has "flat" attribute, it is an ndarray and that should
#         # be returned. Otherwise just return T.
#         if not T:
#             xform = TransformState.makeIdentity()
#         else:
#             if not isinstance(T, TransformState):
#                 try:
#                     mat = Mat4(*T.flat) if hasattr(T, "flat") else Mat4(T)
#                 except TypeError:
#                     mat = None
#                 xform = TransformState.makeMat(mat)
#             else:
#                 xform = T
#         return xform

#     @classmethod
#     def read_node(cls, node):
#         """ Gets 'shape' from Bullet*Shape(s)."""
#         # Get valid node.
#         try:
#             node = node.node()
#         except AttributeError:
#             pass
#         # Get shapes.
#         n_shapes = node.getNumShapes()
#         if n_shapes == 0:
#             # For no shape, return "".
#             bshape = cls("")
#         else:
#             # For 1+ shapes.
#             parent = node.getParent()
#             node.detachNode()
#             vals = []
#             for i in xrange(n_shapes):
#                 # Get shape and shape's matrix.
#                 bulletshape = node.getShape(i)
#                 # Get name and params.
#                 name = cls.shape2name[type(bulletshape)]
#                 params = tuple([getattr(bulletshape, "get%s" % pname)()
#                                 for pname in cls.parameters[name]])
#                 # Get xform.
#                 xform = TransformState.makeMat(node.getShapeMat(i))
#                 # Store.
#                 vals.append((name, params, xform))
#             node.reparentTo(parent)
#             # Create BShape object.
#             bshape = cls(vals)
#         return bshape

#     @classmethod
#     def _init(cls, name, params):
#         """ Initialize one BulletShape."""
#         if not params:
#             params = cls.parameters[name].values()
#         bshape = cls.name2shape[name](*params)
#         return bshape

#     def init(self):
#         """ Build the BulletShapes (shapes) and TransformStates
#         (xforms). We need to apply the node's scale to the parameters
#         of the BulletShape, and also before translating the shapes'
#         xforms."""
#         shapes = []
#         xforms = []
#         for name, params, xform in self:
#             shape = self._init(name, params)
#             shapes.append(shape)
#             xforms.append(xform)
#         return shapes, xforms



class PSO(SSO):
    """ Bullet physics object."""

    type_ = BulletBodyNode
    _prop_tags = ("friction", "restitution", "shape", "deactivation_enabled")
    _res_tags = ("shape",)

    def __init__(self, *args, **kwargs):
        # Converts args so they're appropriate for self.type_.
        if len(args) == 0:
            args = ("",)
        if isinstance(args[0], str):
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

    def add_shape(self, bshapes):
        """ Adds the shape."""
        # Construct the BulletShapes.
        shapes, xforms = bshapes.init()
        for shape, xform in izip(shapes, xforms):
            # Add each to the BulletBodyNode.
            self.node().addShape(shape, xform)

    def create_shape(self):
        """ Initializes a BulletShape(s)."""
        bshapes = BShapeManager(self.get_shape())
        self.add_shape(bshapes)
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
    _prop_tags = ("linear_velocity", "angular_velocity", "mass", "gravity")
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
        ones = Vec3(1, 1, 1)
        shapes = []
        for pso in psos:
            name, parm0, xform0 = BShapeManager._safe_set1(pso.get_shape())
            if name != "Box":
                print("Can't handle that shape: %s" % name)
                BP()
            scale = pso.get_scale(self)
            parm0 = BShapeManager.parameters[name]["HalfExtentsWithMargin"]
            parm1 = (Vec3(*[s * p for s, p in izip(scale, parm0)]),)
            pos = pso.get_pos(self)
            quat = pso.get_quat(self)
            T = TransformState.makePosQuatScale(pos, quat, ones)
            xform1 = T.compose(xform0)
            shape = (name, parm1, xform1)
            shapes.append(shape)
        # Set compound object's shapes tag.
        self.set_shape(shapes)

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
