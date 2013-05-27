""" Picker: view and click objects in a scene."""
# Standard
# External
from libpanda import Vec3, Vec4
from pandac.PandaModules import GeomNode, RenderModeAttrib
from pandac.PandaModules import (CollisionHandlerQueue, CollisionNode,
                                 CollisionRay, CollisionTraverser)
# Project
from scenesim.display.viewer import Viewer
from scenesim.physics.bulletbase import JointManager
from scenesim.objects import PSO


class Picker(Viewer):
    """ View and click objects in a scene."""

    def __init__(self):
        # Parent init.
        super(Picker, self).__init__()
        self.disableMouse()
        # Picker stuff.
        self.marked = None
        self.attached_pairs = []
        self.joints = JointManager()
        self.wire_attrib = RenderModeAttrib.make(
            RenderModeAttrib.MWireframe, 4.)
        self.attachment_colors = (Vec4(0.1, 0.1, 1., 1.),
                                  Vec4(0.1, 0.8, 0.1, 1.),
                                  Vec4(1., 0.1, 1., 1.),
                                  Vec4(1., 0.5, 0.1, 1.),
                                  Vec4(1., 1., 1., 1.))
        self.max_attach = 5
        self.permanent_events += ["mouse1", "mouse3"]

    def init_ssos(self, *args, **kwargs):
        super(Picker, self).init_ssos(*args, **kwargs)
        # Initialize picker.
        pickables = sum((sso.descendants(type_=PSO)
                         for sso in self.ssos), [])
        self.init_picker(pickables)

    def init_physics(self, *args, **kwargs):
        super(Picker, self).init_physics(*args, **kwargs)
        self.joints.bbase = self.bbase

    def goto_sso(self, *args, **kwargs):
        self.clear_attachments()
        super(Picker, self).goto_sso(*args, **kwargs)

    def init_picker(self, pickables):
        # Collision traverser
        self.traverser = CollisionTraverser("traverser")
        # Collision handler
        self.handler = CollisionHandlerQueue()
        # Initialize and set up picker ray node and NodePath
        self.picker = CollisionNode("mouse_ray")
        self.pickerNP = self.camera.attachNewNode(self.picker)
        self.picker.setFromCollideMask(GeomNode.getDefaultCollideMask())
        self.picker_ray = CollisionRay()
        self.picker.addSolid(self.picker_ray)
        self.traverser.addCollider(self.pickerNP, self.handler)
        # Set pickable objs.
        for i, obj in enumerate(pickables):
            obj.setTag("pickable", str(i))
        # Add mouse events.
        self.accept("mouse1", self.clicked, extraArgs=[1])
        self.accept("mouse3", self.clicked, extraArgs=[2])

    def get_picked_obj(self):
        mpos = self.mouseWatcherNode.getMouse()
        self.picker_ray.setFromLens(self.cam.node(), mpos.getX(), mpos.getY())
        self.traverser.traverse(self.render)
        if self.handler.getNumEntries() > 0:
            # This is so we get the closest object
            self.handler.sortEntries()
            entries = self.handler.getEntries()
            for entry in entries:
                picked_obj = entry.getIntoNodePath().findNetTag("pickable")
                if not picked_obj.isEmpty():
                    break
            if picked_obj.isEmpty():
                picked_obj = None
        else:
            picked_obj = None
        return picked_obj

    def clicked(self, button):
        """ Mouse click handler."""
        if self.mouseWatcherNode.hasMouse():
            # Proportion of time spent in this phase so far [0, 1]
            # elapsed = self._get_elapsed()
            # Get picked object
            picked_obj = self.get_picked_obj()
            if picked_obj is not None:
                if self.marked is None:
                    self.marked = picked_obj
                    self.show_marked(picked_obj, True)
                elif picked_obj == self.marked:
                    self.show_marked(picked_obj, False)
                    self.marked = None
                else:
                    f_add = button == 1
                    self.store_attachment(picked_obj, f_add)
                    self.show_marked(self.marked, False)
                    self.marked = None

    def store_attachment(self, picked_obj, f_add):
        """ Stores the attached objects, and draws them."""
        pair = set((self.marked, picked_obj))
        if f_add:
            if (len(self.attached_pairs) < self.max_attach and
                pair not in self.attached_pairs):
                self.attached_pairs.append(pair)
                self.show_attachment(pair, True)
                self.attach_pair(pair, True)
        else:
            try:
                self.attached_pairs.remove(pair)
                self.attach_pair(pair, False)
                self.show_attachment(pair, False)
            except:
                pass

    def clear_attachments(self):
        """ Clear all attachments."""
        if self.marked:
            self.show_marked(self.marked, False)
            self.marked = None
        for pair in self.attached_pairs:
            self.attach_pair(pair, False)
            self.show_attachment(pair, False)
        self.attached_pairs = []

    def show_marked(self, node, f_on):
        """ Turns on/off marked graphic."""
        box = node.find("marked-box")
        if box.isEmpty() and f_on:
            box = self.loader.loadModel("box.bam")
            box.setPos(Vec3(-0.5, -0.5, -0.5))
            box.setName("marked-box")
            box.setAttrib(self.wire_attrib)
            box.setColor(Vec4(0., 0., 0., 1.))
            box.setScale(box.getScale())
            # Exclude object from casting shadows
            box.hide(self.shadow_mask)
            box.reparentTo(node)
        elif not f_on:
            box.removeNode()

    def show_attachment(self, pair, f_on):
        """ Turns on/off attachment graphic."""
        if f_on:
            for node in pair:
                box = self.loader.loadModel("box.bam")
                box.setPos(Vec3(-0.5, -0.5, -0.5))
                box.setName("box")
                box.setAttrib(self.wire_attrib)
                box.setColor(
                    self.attachment_colors[len(self.attached_pairs) - 1])
                box.setScale(box.getScale())
                box.reparentTo(node)
        else:
            for node in pair:
                box = node.find("box")
                box.removeNode()

    def attach_pair(self, pair, f_on):
        """ Adds/removes a fixedjoint between a pair of nodes."""
        key = tuple(sorted(p.node() for p in pair))
        if f_on:
            # Create the joint and add it.
            self.joints[key] = self.joints.make_fixed(*pair)
        else:
            # Remove it.
            del self.joints[key]
