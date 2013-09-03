""" Physics objects."""
from collections import Iterable, OrderedDict
from contextlib import contextmanager
from ctypes import c_float
from functools import wraps
from itertools import izip
##
import numpy as np
import panda3d.bullet as p3b
from libpanda import Mat4, Point3, TransformState, Vec3
from panda3d.bullet import BulletBodyNode, BulletGhostNode, BulletRigidBodyNode
##
from scenesim.objects.sso import SSO


def cast_c_float(func):
    """ Decorator for casting a function's return value to ctypes.f32."""
    def func_c_float(*args, **kwargs):
        return c_float(func(*args, **kwargs)).value
    return func_c_float


class BShapeManager(list):
    """ Subclass of list for storing shapes used for creating
    BulletShapes."""

    name2shape = {
        "Box": p3b.BulletBoxShape,
        "Capsule": p3b.BulletCapsuleShape,
        "Cone": p3b.BulletConeShape,
        "Cylinder": p3b.BulletCylinderShape,
        "Plane": p3b.BulletPlaneShape,
        "Sphere": p3b.BulletSphereShape}
    shape2name = {v: k for k, v in name2shape.iteritems()}
    parameters = {
        "Box": OrderedDict((("HalfExtentsWithMargin", Vec3(0.5, 0.5, 0.5)),)),
        "Capsule": OrderedDict((("Radius", 0.5), ("HalfHeight", 0.5))),
        "Cone": OrderedDict((("Radius", 0.5), ("Height", 1.))),
        "Cylinder": OrderedDict((("Radius", 0.5), ("Height", 1.))),
        "Plane": OrderedDict((("PlaneNormal", Vec3(0, 0, 1)),
                              ("PlaneConstant", 0.))),
        "Sphere": OrderedDict((("Radius", 0.5),))}

    def __init__(self, val=""):
        super(self.__class__, self).__init__(self._safe_set(val))

    def __setitem__(self, key, val):
        super(self.__class__, self).__setitem__(key, self._safe_set1(val))

    def __add__(self, val):
        super(self.__class__, self).__add__(self._safe_set(val))

    def __iadd__(self, val):
        super(self.__class__, self).__iadd__(self._safe_set(val))

    def append(self, val):
        super(self.__class__, self).append(self._safe_set1(val))

    def extend(self, val):
        super(self.__class__, self).extend(self._safe_set(val))

    def insert(self, key, val):
        super(self.__class__, self).insert(key, self._safe_set1(val))

    @classmethod
    def _safe_set1(cls, X):
        """ Standardizes 1-element X."""
        if X == "":
            S = ("",)
        elif isinstance(X, str):
            # X is a string. Put defaults in params and None in xform
            # (which will be made the identity transform).
            S = (X, (), None)
        elif isinstance(X, Iterable) and len(X) != 3:
            raise ValueError("Bad input.")
        else:
            # Make X into tuple S.
            S = tuple(X)
        # Check that the shape is valid.
        if S[0] not in ("",) + tuple(cls.parameters):
            raise ValueError("Bad shape: %s" % S[0])
        if S[0]:
            # Make xform a TransformState.
            xform = cls._xform(S[2])
            if xform is None:
                raise ValueError("Bad xform: %s" % str(S[2]))
            shape = (S[0], S[1], xform)
        else:
            shape = None
        return shape

    def _safe_set(self, X):
        """ Standardizes list of Xs."""
        # Organize into list of tuples, S: [(str, param, TransformState), ...].
        if X == "":
            X = []
        elif isinstance(X, str):
            X = [X]
        elif (isinstance(X, Iterable) and len(X) == 3 and
              isinstance(X[0], str) and self._xform(X[2])):
            X = [X]
        # Verify that X contains valid names and xform matrices.
        shape = []
        for x in X:
            # Standardize the shape.
            s = self._safe_set1(x)
            # If invalid, fail.
            if not s:
                raise ValueError("Bad shape: %s" % str(x))
            shape.append(s)
        return shape

    @classmethod
    def _xform(cls, T):
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
    def read_node(cls, node):
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
            bshape = cls("")
        else:
            # For 1+ shapes.
            parent = node.getParent()
            node.detachNode()
            vals = []
            for i in xrange(n_shapes):
                # Get shape and shape's matrix.
                bulletshape = node.getShape(i)
                # Get name and params.
                name = cls.shape2name[type(bulletshape)]
                params = tuple([getattr(bulletshape, "get%s" % pname)()
                                for pname in cls.parameters[name]])
                # Get xform.
                xform = TransformState.makeMat(node.getShapeMat(i))
                # Store.
                vals.append((name, params, xform))
            node.reparentTo(parent)
            # Create BShape object.
            bshape = cls(vals)
        return bshape

    @classmethod
    def _init(cls, name, params):
        """ Initialize one BulletShape."""
        if not params:
            params = cls.parameters[name].values()
        bshape = cls.name2shape[name](*params)
        return bshape

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


class CPSO(PSO):
    """ Bullet physics object, specialized for compound shapes."""

    def __init__(self, *args, **kwargs):
        if len(args) == 0:
            args = ("compound",)
        super(CPSO, self).__init__(*args, **kwargs)
        self._shapes = []

    @contextmanager
    def _keep_child_tranforms(self):
        """ Remember transforms of existing children to avoid
        center-of-mass shift."""
        parent = self.getParent()
        descendants = self.descendants(depth=[1])
        # Remember child transforms.
        mats = [child.get_mat(parent) for child in descendants]
        yield parent
        # Update transforms of existing children.
        for descendant, mat in izip(descendants, mats):
            descendant.set_mat(parent, mat)

    def _compute_shapes(self):
        """ Computes shapes from self._psos."""
        # Compute mass and center-of-mass.
        masses = []
        poses = []
        for pso in self._psos:
            mass = pso.get_mass()
            pos = pso.get_pos(self)
            if mass == 0.:
                com = pos
                break
            poses.append(pos)
            masses.append(mass)
        else:
            mass = np.sum(masses)
            com = Point3(*(np.sum(np.array(poses).T * masses, axis=-1) / mass))
        self.set_mass(mass)
        with self._keep_child_tranforms() as parent:
            self.set_pos(parent, com)
        # Add shapes from PSOs.
        ones = Vec3(1, 1, 1)
        shapes = []
        for pso in self._psos:
            pso.wrtReparentTo(self)
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
        self._psos.extend(psos)
        self._compute_shapes()

    def remove(self, psos):
        """ Remove sequence of PSOs from compound object."""
        for pso in psos:
            self._psos.remove(pso)
        self._compute_shapes()


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
