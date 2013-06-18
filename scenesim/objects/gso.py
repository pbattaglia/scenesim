""" Graphics objects."""
from functools import wraps
from pdb import set_trace as BP

from panda3d.core import GeomNode, ModelRoot, NodePath
from path import path

from scenesim.display.lightbase import Loader
from scenesim.objects.sso import SSO


class GSOError(Exception):
    pass


class LoaderError(GSOError):
    pass


class GSO(SSO):
    """ Graphics SSO."""

    _prop_tags = ("color", "model")
    _res_tags = ("model",)
    loader = Loader

    def __init__(self, *args, **kwargs):
        ## Using super fails, probably because NodePath is a C++ class.
        # super(GSO, self).__init__(*args, **kwargs)
        SSO.__init__(self, *args, **kwargs)

    @wraps(SSO.get_color, assigned=("__name__", "__doc__"))
    def get_color(self):
        if not self.hasColor():
            self.set_color(0, 0, 0, 1)
        return super(SSO, self).get_color()

    def set_model(self, model):
        self.setPythonTag("model", model)

    def get_model(self):
        if "model" not in self.getPythonTagKeys():
            self.set_model("")
        return self.getPythonTag("model")

    def create_model(self):
        """ Initialize modelnode. GSOs should not have non-resource
        nodes parented to them because 'destroy_model' removes all
        descendant nodes with tag 'model'."""
        model_name = self.get_model()
        try:
            # Load the model from disk.
            node = self.loader.load_model(model_name)
        except NameError:
            # Probably won't enter here, but if so it needs to be debugged.
            BP()
            pass
        else:
            if node is None:
                raise LoaderError("Could not find model: %s" % model_name)
            node.setName(path(model_name).basename())
            NodePath(node).reparentTo(self)
            self.clear_materials()
            self.setTag("resource", "model")

    def delete_model(self):
        """ Destroy model nodes that are descendants and have tag 'model'."""
        if not self.isEmpty() and self.getTag("resource") == "model":
            nodes = self.descendants(depths=slice(1, None))
            for node in nodes:
                if isinstance(node.node(), ModelRoot):
                    node.clearModelNodes()
                    node.flattenStrong()
                    node.removeNode()
                elif isinstance(node.node(), GeomNode):
                    node.removeNode()
                if not node.isEmpty():
                    node.clearTag("resource")

    def clear_materials(self):
        """ Clears the material attributes from the node. """
        for mat in self.findAllMaterials():
            mat.clearAmbient()
            mat.clearDiffuse()
        self.clearMaterial()
        self.clearTexture()
