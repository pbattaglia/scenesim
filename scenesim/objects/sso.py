""" Basic SceneSim objects."""
import cPickle as pickle
from collections import Iterable
from functools import partial
from itertools import izip

from pandac.PandaModules import NodePath, NodePathCollection, PandaNode
from path import path

from scenesim.lib import combomethod


class SSO(NodePath):
    """ Subclass of NodePath with SSO-specific functionality."""

    type_ = PandaNode
    _prop_tags = ("pos", "quat", "scale", "name")
    _res_tags = ()

    def __init__(self, *args, **kwargs):
        props = kwargs.pop("props", {})
        other = kwargs.pop("other", {})
        if len(args) == 0:
            args = ("",)
        super(SSO, self).__init__(*args, **kwargs)
        if isinstance(args[0], str):
            self.setPythonTag("sso", self.__class__)
        self.apply_prop(props, other=other)

    @classmethod
    def cast(cls, node):
        """ Return a node casted to this class type, and set its
        python tag."""
        node.setPythonTag("sso", cls)
        return cls(node)

    @classmethod
    def from_tag(cls, node):
        """ Return a node whose type is that which is contained in its
        python tag."""
        obj = (node.getPythonTag("sso")(node)
               if node.hasPythonTag("sso") else cls(node))
        return obj

    @classmethod
    def default(cls):
        """ Gets a property dictionary with default values."""
        sso = cls()
        return sso.read_prop()

    @property
    def prop_tags(self):
        """ (property)."""
        gen = (list(getattr(c, "_prop_tags", []))
               for c in self.__class__.mro())
        tags = tuple(set(sum(gen, [])))
        return tags

    @property
    def res_tags(self):
        """ (property)."""
        gen = (list(getattr(c, "_res_tags", [])) for c in self.__class__.mro())
        tags = tuple(set(sum(gen, [])))
        return tags

    def apply_prop(self, props, other=None):
        """ Set state properties.

        Parameters
        ----------
        prop : dict
            A dictionary containing new state property values.
        other : NodePath (default=self.getParent())
            The node which the new state is set relative to.

        """
        if other is None:
            other = self.getParent()
        # Iterate through the property dict, setting each.
        for tag, prop in props.iteritems():
            # Get the setter.
            try:
                # From the NodePath.
                method = getattr(self, "set_%s" % tag)
            except AttributeError as err:
                try:
                    # From the PandaNode.
                    method = getattr(self.node(), "set_%s" % tag, None)
                except AttributeError:
                    raise err
                else:
                    print("You should add method set_%s to %s" %
                          (tag, type(self)))
            if method:
                # Do the set.
                try:
                    # With other.
                    method(other, prop)
                except TypeError:
                    # Without other.
                    method(prop)

    def read_prop(self, other=None):
        """ Returns a dictionary of state properties.

        Note that the dictionary values are copies of the prop, not
        references, so modifications to the returned dictionary will not
        change self.

        Parameters
        ----------
        other : NodePath (default=self.getParent())
            The node which the returned prop is relative to.

        Returns
        -------
        prop : dict
            A dictionary representation of the current prop.

        """
        if other is None:
            other = self.getParent()
        props = {}
        # Iterate through the tags, getting each property.
        for tag in self.prop_tags:
            try:
                # Try getting the getter from the NodePath
                method = getattr(self, "get_%s" % tag)
            except AttributeError as err:
                # If that fails, try getting it from the PandaNode.
                try:
                    method = getattr(self.node(), "get_%s" % tag)
                except AttributeError:
                    # If that also fails, raise the original exception.
                    raise err
                else:
                    print("You should add method get_%s to %s" %
                          (tag, type(self)))
            # Do the get.
            try:
                # With other.
                props[tag] = method(other)
            except TypeError:
                # Without other.
                props[tag] = method()
        return props

    def init_resources(self, tags=None):
        """ Initializes a node's resources."""
        # Destroy previous resources.
        self.destroy_resources(tags=tags)
        if tags is None:
            tags = self.res_tags
        for tag in tags:
            try:
                method = getattr(self, "create_%s" % tag)
            except AttributeError:
                pass
            else:
                method()

    def destroy_resources(self, tags=None):
        """ Destroys the resources created by init_resources()."""
        if not self.isEmpty():
            if tags is None:
                tags = self.res_tags
            for tag in tags:
                try:
                    getattr(self, "delete_%s" % tag)()
                except AttributeError:
                    pass

    @staticmethod
    def _has_type(type_, node):
        """ Returns True if `node`'s sso type is subclass of element
        in `type_`."""
        nt = node.getPythonTag("sso")
        return type_ is None or isinstance(nt, type) and issubclass(nt, type_)

    @staticmethod
    def _has_name(names, node):
        """ Returns True if `node`'s name starts with element in `names`."""
        return names is None or any(node.getName().startswith(name)
                                    for name in names)

    @classmethod
    def _filter(cls, node, type_, names):
        """ Returns True if node passes type and name filters."""
        if names is not None and isinstance(names, str):
            names = (names,)
        filts = (partial(cls._has_type, type_), partial(cls._has_name, names))
        return all(filt(node) for filt in filts)

    @classmethod
    def filter(cls, nodes, type_=None, names=None):
        """ Filters out nodes whose sso type_ are not elements of
        `type_` or do not start with an element of `names`."""
        if not isinstance(nodes, Iterable):
            nodes = (nodes,)
        return tuple(n for n in nodes if cls._filter(n, type_, names))

    def descendants(self, depths=slice(None), type_=None, names=None):
        """ Returns a list of descendants of this node. All if 'depth' is
        None, down to 'depths' (if int), only at 'depth' levels (if
        Iterable), or in a range (if slice)."""
        # Filter out those that don't meet 'depth' criteria.
        # Different in/out test depending on 'depth'.
        if isinstance(depths, slice):
            rng = xrange(*depths.indices(NodePath.getMaxSearchDepth()))
        elif not isinstance(depths, Iterable):
            rng = xrange(depths + 1)
        else:
            rng = depths

        # Get all descendants. Filter type_ and names.
        dsc = [n for n in (self.from_tag(node)
                           for node in self.findAllMatches("**"))
               if self._filter(n, type_, names) and n.getNumNodes() - 1 in rng]
        return dsc

    def tree(self):
        """ Non-recursive descendant tree getter. Outputs the list of
        nodes and partial order tree structure. Breadth-first."""
        self_key = self.getKey()
        nodes = self.descendants()
        nidx = {n.getKey(): i for i, n in enumerate(nodes)}
        porder = tuple(nidx[n.getParent().getKey()] + 1
                       if n.getKey() != self_key else 0 for n in nodes)
        return nodes, porder

    def tree_prop(self):
        """ Input node and return the property dictionaries and a partial
        ordering of its descendants."""
        nodes, porder = self.tree()
        # Get the props.
        props = tuple(n.read_prop() for n in nodes)
        return props, porder

    def state(self):
        """ Input node and return the state: type_s, nodes and partial
        ordering of its descendants."""
        nodes, porder = self.tree()
        types = tuple(node.__class__ for node in nodes)
        return types, nodes, porder

    def state_prop(self):
        """ Input node and return the state: type_s, property dicts,
        and partial ordering of its descendants."""
        types, nodes, porder = self.state()
        props = tuple(node.read_prop() for node in nodes)
        return types, props, porder

    @classmethod
    def connect_tree(cls, nodes, porder, wrt=False):
        """ Connect up a tree by reparenting nodes according to
        `porder`. Return the top node."""
        for n, p in zip(nodes, porder):
            if p > 0:
                if wrt:
                    n.wrtReparentTo(nodes[p - 1])
                else:
                    n.reparentTo(nodes[p - 1])
        # Top node.
        top = cls.from_tag(nodes[0].getTop())
        return top

    def disconnect_tree(self, wrt=False):
        """ Disconnect a tree. wrt keeps state intact."""
        nodes, porder = self.tree()  # TODO: add type_ filter?
        npc = NodePathCollection(nodes)
        if wrt:
            npc.wrtReparentTo(SSO("disconnected_top"))
        npc.detach()

    @classmethod
    def build_tree(cls, X, porder, types=None):
        """ Input list of states/NodePaths and partial order, and
        construct a NodePath tree."""
        # Construct list of NodePaths, constructing them from states
        # if need be.
        if types:
            nodes = [type_(x) if isinstance(x, NodePath) else type_(props=x)
                     for x, type_ in izip(X, types)]
        else:
            nodes = [x if isinstance(x, NodePath) else cls(props=x) for x in X]
        # Connect the nodes according to `porder`. node is the top.
        node = cls.connect_tree(nodes, porder)
        return node

    def store_tree(self):
        """ Store a node tree's prop. Input a NodePath and return a dict
        that can be restored."""
        cache = Cache.store(self)
        return cache

    def copy(self):
        """ Copy a node tree and return new top node."""
        types, props, porder = self.state_prop()  # TODO: add type_ filter?
        node = self.build_tree(props, porder, types=types)
        return node

    def init_tree(self, tags=None):
        """ Inits this node tree's resources."""
        # Get all descendants.
        nodes = self.descendants(type_=SSO)
        for node in nodes:
            node.init_resources(tags=tags)

    def destroy_tree(self, tags=None):
        """ Destroys this node tree's resources."""
        # Get all descendants.
        nodes = self.descendants(type_=SSO)
        for n in nodes:
            n.destroy_resources(tags=tags)

    def dumps(self, other=None):
        """ Dump property into pickled string representation."""
        state = (self.__class__, self.read_prop(other=other))
        serial = pickle.dumps(state)
        return serial

    def dump(self, F, other=None):
        """ Dump property into a file F."""
        state = (self.__class__, self.read_prop(other=other))
        if isinstance(F, file):
            pickle.dump(state, F)
        else:
            with path(F).open("w") as fid:
                pickle.dump(state, fid)

    @staticmethod
    def reads(serial):
        """ Read type and property dict from pickled string."""
        type_, props = pickle.loads(serial)
        return type_, props

    @staticmethod
    def read(F):
        """ Read type and property dict from pickled file."""
        if isinstance(F, file):
            type_, props = pickle.load(F)
        else:
            with path(F).open("r") as fid:
                type_, props = pickle.load(fid)
        return type_, props

    @combomethod
    def _load(combo, type_, props, other=None):
        """ Utility for actually loading an SSO."""
        if isinstance(combo, type):
            sso = type_(props=props, other=other)
        else:
            sso = type_.cast(combo)
            sso.apply_prop(props, other=other)
        return sso

    @combomethod
    def loads(combo, serial, other=None):
        """ Load pickled property string and return sso. (combomethod)"""
        type_, props = combo.reads(serial)
        sso = combo._load(type_, props, other=other)
        return sso

    @combomethod
    def load(combo, F, other=None):
        """ Load pickled file containing property string and return
        sso. (combomethod)"""
        type_, props = combo.read(F)
        sso = combo._load(type_, props, other=other)
        return sso

    def save_tree(self, F):
        """ Saves tree to file or path F."""
        state = self.state_prop()  # TODO: add type_ filter?
        # Save to disk.
        if isinstance(F, file):
            pickle.dump(state, F)
        else:
            with path(F).open("w") as fid:
                pickle.dump(state, fid)

    @classmethod
    def load_tree(cls, F):
        """ Loads tree from file or path F."""
        # Load from disk.
        if isinstance(F, file):
            types, props, porder = pickle.load(F)
        else:
            with path(F).open("r") as fid:
                types, props, porder = pickle.load(fid)
        ssos = (cls._load(type_, prop) for type_, prop in izip(types, props))
        # Build the tree. node is the top.
        top = cls.build_tree(ssos, porder)
        return top


class Cache(object):
    """ Stores a node tree's nodes, properties, and partial order,
    which can then be restored later."""

    def __init__(self):
        self._nodes = []
        self._props = []
        self._porder = []

    @classmethod
    def store(cls, node):
        """ Store a node tree's prop. Input a NodePath and return a dict
        that can be restored."""
        self = cls()
        # Get the nodes and partial.
        self._nodes, self._porder = node.tree()
        # Build the cache. The keys are the node names and the values
        # are the nodes and props.
        self._props = [SSO.from_tag(n).read_prop() for n in self._nodes]
        return self

    def restore(self):
        """ Restores a cached node tree's prop. Input a cache dict and
        restore the node props."""
        # Iterate over each node key in the cache dict, setting the node
        # and its tag to the prop value.
        for node, props in izip(self._nodes, self._props):
            node.apply_prop(props)
        top = SSO.connect_tree(self._nodes, self._porder)
        return top
