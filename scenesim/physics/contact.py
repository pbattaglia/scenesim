""" Identifies contiguous subsets of objects using Bullet's collision
detection."""
# Standard
from collections import OrderedDict
from contextlib import contextmanager
from itertools import chain, combinations
# External
from libpanda import Vec3
import networkx as nx
import numpy as np
from Polygon.Utils import convexHull
from scenesim.objects.pso import PSO
from scenesim.physics.bulletbase import BulletBase
# Project
#
from pdb import set_trace as BP


def powerset(s):
    "powerset([1,2,3]) --> () (1,) (2,) (3,) (1,2) (1,3) (2,3) (1,2,3)"
    return chain.from_iterable(combinations(s, r) for r in xrange(len(s) + 1))


def get_bb(mats):
    """ Get the bounding box of a set of unit-sized objects
    transformed by `mats'."""
    verts = np.array([
        [0, 0, 0, 1],
        [0, 1, 0, 1],
        [1, 0, 0, 1],
        [1, 1, 0, 1],
        [0, 0, 1, 1],
        [0, 1, 1, 1],
        [1, 0, 1, 1],
        [1, 1, 1, 1]]) + np.array([-0.5, -0.5, -0.5, 0])
    bbs = np.vstack(np.dot(verts, mat)[:, :3] for mat in mats)
    bb = np.array((bbs.min(axis=0), bbs.max(axis=0)))
    return bb


class Contact(object):
    """ Defines `Contact` objects, which provide a nicer interface to
    Bullet's ContactResults."""

    def __init__(self, contact_result):
        self._contact_result = contact_result
        self._N = None
        self._points = None
        self._normal = None
        self._below = None

    @property
    def N(self):
        """ Returns the number of contacts."""
        if self._N is None:
            self._N = self._contact_result.getNumContacts()
        return self._N

    @property
    def points(self):
        """ Returns list of points from each of the contacts."""
        if self._points is None:
            self._points = []
            for contact in self._contact_result.getContacts():
                mpoint = contact.getManifoldPoint()
                pa = mpoint.getPositionWorldOnA()
                pb = mpoint.getPositionWorldOnB()
                self._points.append((pa + pb) / 2.)
        return self._points

    @property
    def normal(self):
        """ Mean normal on A."""
        if self._normal is None:
            bns = np.array([-contact.getManifoldPoint().getNormalWorldOnB()
                            for contact in self._contact_result.getContacts()])
            self._normal = bns.mean(axis=0)
        return self._normal

    @property
    def below(self):
        """ Indicates whether body A is below."""
        if self._below is None:
            self._below = self.normal[2] > 0
        return self._below

    @property
    def chull(self):
        """ Return convex hull of points."""
        if self._chull is None:
            self._chull = convexHull(self.points).contour(0)
        return self._chull


class Detector(object):
    """ Monitors contacts between bodies."""

    def __init__(self, world, bodies):
        self.world = world
        self.bodies = bodies
        self.n_bodies = len(bodies)
        self.clear()

    def clear(self):
        """ Clear all of the cached values."""
        self._pairs = None
        self._contacts = None
        self._points = None
        self._normals = None
        self._belows = None
        self._matrix = None
        self._N = None

    def _test(self, a, b):
        """ Perform actual pairwise contact tests."""
        return Contact(self.world.contactTestPair(a.node(), b.node()))

    def test(self):
        """ Test for contacts and store in `self._pairs`."""
        self.clear()
        self._pairs = {}
        for (i, a), (j, b) in combinations(enumerate(self.bodies), 2):
            contact = self._test(a, b)
            self._pairs[i, j] = contact
            self._pairs[j, i] = contact

    @property
    def pairs(self):
        """ Returns contact info between `self.bodies`."""
        if self._pairs is None:
            self.test()
        return self._pairs

    @property
    def contacts(self):
        """ List of contacts and the indices of the involved bodies."""
        if self._contacts is None:
            self._contacts = OrderedDict(
                sorted(((i, j), c) for (i, j), c in self.pairs.iteritems()
                       if c.N > 0 and i < j))
        return self._contacts

    @property
    def points(self):
        """ Returns `points` corresponding to each contact and the
        indices of the involved bodies."""
        if self._points is None:
            self._points = OrderedDict((
                ((i, j), c.points) for (i, j), c in self.contacts.iteritems()))
        return self._points

    @property
    def normals(self):
        """ Returns `normal`s corresponding to each contact and the
        indices of the involved bodies."""
        if self._normals is None:
            self._normals = OrderedDict((
                ((i, j), c.normal) for (i, j), c in self.contacts.iteritems()))
        return self._normals

    @property
    def belows(self):
        """ Returns `below` indicators corresponding to each contact and
        the indices of the involved bodies."""
        if self._belows is None:
            self._belows = OrderedDict((
                ((i, j), c.below) for (i, j), c in self.contacts.iteritems()))
        return self._belows

    @property
    def matrix(self):
        """ Returns a matrix that stores the number of contacts
        between every pair of bodies."""
        if self._matrix is None:
            self._matrix = np.zeros([self.n_bodies] * 2, dtype="uint16")
            for (i, j), contact in self.contacts.iteritems():
                self._matrix[i, j] = contact.N
        return self._matrix

    @property
    def N(self):
        """ Returns the number of contacts."""
        if self._N is None:
            self._N = len(self.contacts)
        return self._N

    # @classmethod
    # def detect(cls, scene, margin):
    #     """ Increase the node's scale by `margin`."""
    #     # Store initial state.
    #     cache = scene.store_tree()
    #     # Scale the physical bodies.
    #     top = scene.getTop()
    #     nodes = scene.descendants(type_=PSO)
    #     for node in nodes:
    #         node.setScale(top, node.getScale(top) + Vec3(*margin))
    #     scene.init_tree(tags=("shape",))
    #     bbase = BulletBase()
    #     bbase.init()
    #     bbase.gravity = (0, 0, 0)
    #     self = cls(bbase.world, nodes)
    #     bbase.attach(nodes)
    #     self.test()
    #     bbase.remove(nodes)
    #     bbase.destroy()
    #     scene.destroy_tree(tags=("shape",))
    #     cache.restore()
    #     return self


class ContactDetector(Detector):

    def __init__(self, world, scene, margin=None, type_=PSO, names=None):
        self.scene = scene
        self.top = self.scene.getTop()
        # Store initial state.
        self.cache = self.scene.store_tree()
        if margin is None:
            margin = (0.01, 0.01, 0.01)
        self.margin = Vec3(*margin)
        bodies = self.scene.descendants(type_=type_, names=names)
        super(ContactDetector, self).__init__(world, bodies)
        self.detect()

    def detect(self):
        """ Increase the node's scale by `margin`."""
        self.clear()
        # Scale the physical bodies.
        for body in self.bodies:
            body.setScale(self.top, body.getScale(self.top) + self.margin)
        self.scene.init_tree(tags=("shape",))
        for b in self.bodies:
            self.world.attach(b.node())
        self.test()
        for b in self.bodies:
            self.world.remove(b.node())
        self.scene.destroy_tree(tags=("shape",))
        self.cache.restore()


class Parser(object):
    """ Computes subgroups based on contact relationships. Also
    computes surface types."""

    def __init__(self, contacts, bodies):
        self.contacts = contacts
        self.bodies = bodies
        self.n_bodies = len(self.bodies)
        self.clear()

    def clear(self):
        self._surfaces = None
        self._support = None
        self._relations = None
        self._bottom_idx = None
        self._bottom_ibody = None
        self._bottom_bodies = None
        self._graph = None
        self._subgroups = None

    @property
    def surfaces(self):
        """ Returns the bottom and top Z surfaces for each element of
        `self.bodies`."""
        if self._surfaces is None:
            self._surfaces = np.round(
                [sorted(get_bb([np.array(body.getMat())])[:, 2])
                 for body in self.bodies], decimals=5)
        return self._surfaces

    @property
    def support(self):
        """
        self.support[i, j] = 1 => `i` is supporting `j`.
        self.support[i, j] = 2 => `j` is supporting `i`.
        self.support[i, j] = 3 => `i` and `j` are not supporting each other.
        """
        if self._support is None:
            eps = 0.01
            t1 = np.abs(self.surfaces[:, None, 1] -
                        self.surfaces[None, :, 0]) < eps
            t2 = np.abs(self.surfaces[:, None, 0] -
                        self.surfaces[None, :, 1]) < eps
            t3 = ~(t1 | t2)
            self._support = np.zeros([self.n_bodies] * 2, dtype="uint8")
            self._support[t1] = 1
            self._support[t2] = 2
            self._support[t3] = 3
        return self._support

    @property
    def relations(self):
        """ Returns a matrix that identifies the contact types."""
        if self._relations is None:
            self._relations = np.zeros([self.n_bodies] * 2, dtype="uint8")
            for i, j in self.contacts:
                self._relations[i, j] = self.support[i, j]
        return self._relations

    @property
    def bottom_idx(self):
        """ Returns the indices of the contact pair corresponding to
        the bodies that are on the bottom of each pair."""
        if self._bottom_idx is None:
            self._bottom_idx = OrderedDict()
            for (i, j), contact in self.contacts.iteritems():
                bodies = (self.bodies[i], self.bodies[j])
                s = self.support[i, j]
                if s in (0, 3):
                    idx = bodies[0].getPos()[2] - bodies[1].getPos()[2] > 0.
                else:
                    idx = 2 - s
                self._bottom_idx[i, j] = idx
        return self._bottom_idx

    @property
    def bottom_ibody(self):
        """ Returns the index of the body that is on the bottom of
        each contact."""
        if self._bottom_ibody is None:
            self._bottom_ibody = OrderedDict((
                ((i, j), (i, j)[idx])
                for (i, j), idx in self.bottom_idx.iteritems()))
        return self._bottom_ibody

    @property
    def bottom_bodies(self):
        """ Returns the body that is on the bottom of each contact."""
        if self._bottom_bodies is None:
            self._bottom_bodies = OrderedDict((
                ((i, j), self.bodies[ibody])
                for (i, j), ibody in self.bottom_ibody.iteritems()))
        return self._bottom_bodies


    @property
    def graph(self):
        """ Returns `networkx.Graph` object that represents the
        relations matrix."""
        if self._graph is None:
            self._graph = nx.from_numpy_matrix(self.relations)
        return self._graph

    @property
    def subgroups(self):
        """ List of all subgroups (subgraphs) of the graph."""
        if self._subgroups is None:
            ps = powerset(self.graph.nodes())
            sgs = (g for g in (self.graph.subgraph(s) for s in ps)
                   if g.order() > 0 and nx.is_connected(g))
            self._subgroups = list(sg.nodes() for sg in sgs)
        return self._subgroups
