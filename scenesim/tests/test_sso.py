""" nosetests for scenesim.objects.sso/gso/pso."""
from contextlib import contextmanager
from ctypes import c_float
from itertools import izip
import random

from libpanda import Mat4, Point3, Quat, TransformState, Vec3, Vec4
from panda3d.bullet import BulletBoxShape, BulletRigidBodyNode, BulletShape
from pandac.PandaModules import GeomNode, ModelRoot, NodePath, PandaNode
from path import path

from scenesim.objects.gso import GSO
from scenesim.objects.pso import GHSO, PSO, RBSO, cast_c_float
from scenesim.objects.sso import Cache, SSO


resource_types = (BulletShape, GeomNode, ModelRoot)


@contextmanager
def tmpfile():
    """ Utility for making random files."""
    random.seed()
    done = False
    while not done:
        name = "%d_tmpfile" % (random.random() * 1e30)
        pth = path(name)
        done = not pth.isfile()
    try:
        yield pth
    except KeyboardInterrupt:
        pass
    except SystemExit as err:
        if err.code != 0:
            raise
    finally:
        # Delete temporary file.
        pth.unlink()


def test_sso():
    obj = SSO("sso")
    assert isinstance(obj.node(), PandaNode)


def test_gso():
    obj = GSO("gso")
    assert isinstance(obj.node(), PandaNode)


def test_rbso():
    obj = RBSO("rbso")
    assert isinstance(obj.node(), BulletRigidBodyNode)


def test_init():
    np = NodePath("np")
    np.setPythonTag("sso", GSO)
    npsso = SSO(np)
    assert isinstance(npsso, NodePath)
    assert isinstance(npsso, SSO)


def test_name_immutability():
    sso = SSO("foo")
    before = sso.read_prop()
    before["name"] = "bar"
    assert sso.read_prop()["name"] != before["name"]


def test_scale_immutability():
    sso = SSO("foo")
    before = sso.read_prop()
    before["scale"][0] = 10
    assert sso.read_prop()["scale"] != before["scale"]


def test_quat_immutability():
    sso = SSO("foo")
    before = sso.read_prop()
    before["quat"][1] = 0.55
    assert sso.read_prop()["quat"] != before["quat"]


def test_pos_immutability():
    sso = SSO("foo")
    before = sso.read_prop()
    before["pos"][0] = 10
    assert sso.read_prop()["pos"] != before["pos"]


def test_model_immutability():
    sso = GSO("foo")
    before = sso.read_prop()
    before["model"] = "bar"
    assert sso.read_prop()["model"] != before["model"]


def test_shape_immutability():
    sso = PSO(BulletRigidBodyNode("foo"))
    before = sso.read_prop()
    before["shape"] = "Sphere"
    assert sso.read_prop()["shape"] != before["shape"]


def test_default():
    props = GSO.default()
    assert props["pos"] == Point3(0, 0, 0)


def test_prop_tags():
    sso = SSO("sso")
    gso = GSO("gso")
    brso = RBSO("pso")
    assert set(sso.prop_tags) == set(SSO._prop_tags)
    assert set(gso.prop_tags) == set(SSO._prop_tags + GSO._prop_tags)
    assert set(brso.prop_tags) == set(SSO._prop_tags + PSO._prop_tags +
                                       RBSO._prop_tags)


def test_res_tags():
    sso = SSO("sso")
    gso = GSO("gso")
    brso = RBSO("pso")
    assert set(sso.res_tags) == set(SSO._res_tags)
    assert set(gso.res_tags) == set(SSO._res_tags + GSO._res_tags)
    assert set(brso.res_tags) == set(SSO._res_tags + PSO._res_tags +
                                      RBSO._res_tags)


def test_reads_dumps():
    sso = SSO("foo")
    dump = sso.dumps()
    read = SSO.reads(dump)
    assert (sso.__class__, sso.read_prop()) == read


def test_loads_dumps():
    sso = SSO("foo")
    dump = sso.dumps()
    sso2 = SSO.loads(dump)
    assert (sso.__class__, sso.read_prop()) == (sso2.__class__,
                                                sso2.read_prop())


def test_read_dump():
    sso = SSO("foo")
    with tmpfile() as pth:
        with pth.open("w") as fid:
            sso.dump(fid)
        with pth.open() as fid:
            read = SSO.read(fid)
    assert (sso.__class__, sso.read_prop()) == read


def test_load_dump():
    sso = SSO("foo")
    with tmpfile() as pth:
        with pth.open("w") as fid:
            sso.dump(fid)
        with pth.open() as fid:
            sso2 = SSO.load(fid)
    assert (sso.__class__, sso.read_prop()) == (sso2.__class__,
                                                sso2.read_prop())


def test_apply_prop_read_prop_SSO():
    sso = SSO("foo")
    other = SSO("other")
    other.setPos(100, 200, 300)
    other.setHpr(23, 20, 100)
    other.setScale(6, 2, 9)
    prop0 = {"name": "testname",
             "pos": Point3(1, 2, 3),
             "quat": Quat(2 ** 0.5, 2 ** 0.5, 0, 0),
             "scale": Vec3(10, 9, 8),
             }
    sso.setName(prop0["name"])
    sso.setPos(prop0["pos"])
    sso.setQuat(prop0["quat"])
    sso.setScale(prop0["scale"])
    assert prop0 == sso.read_prop()
    oprop = sso.read_prop(other=other)
    sso.wrtReparentTo(other)
    assert sso.read_prop() == oprop


def test_init_resources_model_shape():
    gso = GSO("bar")
    gso.set_model("smiley.egg")
    gso.init_resources()
    gnodes = gso.descendants(depths=slice(1, None))
    pso = RBSO("bar")
    pso.set_shape("Box")
    pso.init_resources()
    assert any(isinstance(n.node(), resource_types) for n in gnodes)
    assert pso.node().getNumShapes() > 0


def test_destroy_resources_model_shape():
    gso = GSO("bar")
    gso.set_model("smiley.egg")
    gso.init_resources()
    gso.destroy_resources()
    gnodes = gso.descendants(depths=slice(1, None))
    pso = RBSO("bar")
    pso.set_shape("Box")
    pso.init_resources()
    pso.destroy_resources()
    assert not any(isinstance(n.node(), resource_types) for n in gnodes)
    assert pso.node().getNumShapes() == 0


def test_descendants():
    n = 8
    types = (SSO, SSO, SSO, SSO, RBSO, RBSO, RBSO, GSO)
    ssos = [type_.cast(type_(str(i))) for i, type_ in enumerate(types)]
    partial = [0, 1, 1, 2, 3, 3, 5, 6]
    for c, p in izip(ssos, partial):
        if p > 0:
            c.reparentTo(ssos[p - 1])
    assert n == len(ssos[0].descendants())
    assert 6 == len(ssos[0].descendants(depths=2))
    assert 4 == len(ssos[0].descendants(depths=[1, 3]))
    assert 8 == len(ssos[0].descendants(type_=SSO))
    assert 1 == len(ssos[0].descendants(type_=GSO))
    assert 2 == len(ssos[0].descendants(depths=slice(1, 3), type_=RBSO))


def test_cast():
    sso = SSO.cast(NodePath("sso"))
    gso = GSO.cast(NodePath("gso"))
    pso = PSO.cast(NodePath("pso"))
    ghso = GHSO.cast(NodePath("ghso"))
    rbso = RBSO.cast(NodePath("rbso"))
    assert isinstance(sso, NodePath)
    assert isinstance(sso, SSO)
    assert isinstance(gso, NodePath)
    assert isinstance(gso, SSO)
    assert isinstance(gso, GSO)
    assert isinstance(ghso, NodePath)
    assert isinstance(ghso, SSO)
    assert isinstance(ghso, PSO)
    assert isinstance(ghso, GHSO)
    assert isinstance(rbso, NodePath)
    assert isinstance(rbso, SSO)
    assert isinstance(rbso, PSO)
    assert isinstance(rbso, RBSO)


def test_from_tag():
    np = NodePath("sso")
    np.setPythonTag("sso", SSO)
    np2 = NodePath("gso")
    np2.setPythonTag("sso", GSO)
    assert not isinstance(np, SSO)
    assert not isinstance(np2, GSO)
    assert isinstance(SSO.from_tag(np), SSO)
    assert isinstance(GSO.from_tag(np2), GSO)


def test_tree():
    n = 8
    ssos = [SSO(str(i)) for i in xrange(n)]
    partial = [0, 1, 1, 2, 3, 3, 5, 6]
    for c, p in izip(ssos, partial):
        if p > 0:
            c.reparentTo(ssos[p - 1])
    ssos2, partial2 = ssos[0].tree()
    assert sorted(ssos) == sorted(ssos2)
    # How should partial orders be compared?


def test_tree_prop():
    n = 8
    ssos = [SSO(str(i)) for i in xrange(n)]
    partial = [0, 1, 1, 2, 3, 3, 5, 6]
    # for c, p in izip(ssos, partial):
    #     if p > 0:
    #         c.reparentTo(ssos[p - 1])
    SSO.connect_tree(ssos, partial)
    ssos.append(NodePath("last"))
    ssos[-1].reparentTo(ssos[-2])
    props = [SSO.cast(c).read_prop() for c in ssos]
    props2, partial2 = ssos[0].tree_prop()
    assert sorted(props) == sorted(props2)
    # How should partial orders be compared?


def test_connect_tree():
    n = 4
    ssos = [SSO(str(i)) for i in xrange(n)]
    partial = [0, 1, 1, 2]
    top = SSO.connect_tree(ssos, partial)
    top2 = ssos[0]
    ssos[1].reparentTo(ssos[0])
    ssos[2].reparentTo(ssos[0])
    ssos[3].reparentTo(ssos[1])
    assert top.tree() == top2.tree()


def test_disconnect_tree():
    n = 8
    ssos = [SSO(str(i)) for i in xrange(n)]
    partial = [0, 1, 1, 2, 3, 3, 5, 6]
    top = SSO.connect_tree(ssos, partial)
    top.disconnect_tree()
    assert not any(sso.hasParent() for sso in ssos)


def test_build_tree():
    n = 4
    ssos = [SSO(str(i)) for i in xrange(n)]
    props = [SSO(str(i)).read_prop() for i in xrange(n)]
    partial = [0, 1, 1, 2]
    top = SSO.build_tree(ssos, partial)
    top2 = SSO.build_tree(props, partial)
    top3 = ssos[0]
    ssos[1].reparentTo(ssos[0])
    ssos[2].reparentTo(ssos[0])
    ssos[3].reparentTo(ssos[1])
    assert top.tree() == top3.tree()
    assert top.tree_prop() == top2.tree_prop()


def test_store_restore_tree():
    sso = GSO("foo")
    sso2 = GSO("bar")
    sso.set_model("smiley.egg")
    sso2.set_model("smiley.egg")
    sso2.reparentTo(sso)
    cache = sso.store_tree()
    sso3 = cache.restore()
    assert sso.tree() == sso3.tree()


def test_copy():
    sso = SSO("foo")
    sso2 = SSO("bar")
    sso2.reparentTo(sso)
    assert sso.copy().tree_prop() == sso.tree_prop()


def test_save_load_tree():
    sso = SSO("foo")
    sso2 = SSO("bar")
    sso2.reparentTo(sso)
    with tmpfile() as pth:
        with pth.open("w") as fid:
            sso.save_tree(fid)
        sso3 = SSO.load_tree(pth)
        with pth.open() as fid:
            sso4 = SSO.load_tree(fid)
        sso.save_tree(pth)
        sso5 = SSO.load_tree(pth)
        with pth.open() as fid:
            sso6 = SSO.load_tree(fid)
    assert (sso.tree_prop() == sso3.tree_prop() == sso4.tree_prop() ==
            sso5.tree_prop() == sso6.tree_prop())


def test_init_tree():
    sso = GSO("foo")
    sso2 = GSO("bar")
    sso.set_model("smiley.egg")
    sso2.set_model("smiley.egg")
    sso2.reparentTo(sso)
    sso.init_tree()
    nodes = sso.descendants()
    assert any(isinstance(n.node(), resource_types) for n in nodes)


def test_destroy_tree():
    sso = GSO("foo")
    sso2 = GSO("bar")
    sso.set_model("smiley.egg")
    sso2.set_model("smiley.egg")
    sso2.reparentTo(sso)
    nodes = sso.descendants()
    for n in nodes:
        n.init_resources()
    sso.destroy_tree()
    assert not any(isinstance(n.node(), resource_types) for n in nodes)


def test_store_restore():
    sso = GSO("foo")
    sso.set_model("smiley.egg")
    cache = Cache.store(sso)
    sso2 = cache.restore()
    assert sso.tree() == sso2.tree()


## PSO, RBSO
def test_cast_c_float():
    def f():
        return 1. / 9.
    g = cast_c_float(f)
    assert f() == 1. / 9.
    assert g() != 1. / 9.
    assert g() == c_float(1. / 9.).value


def test_pso_set_shape():
    shape = ("Box", (Vec3(4, 1, 7),), TransformState.makePos((2, 4, 6)))
    obj = RBSO("rso")
    obj.set_shape(shape)
    assert obj.getPythonTag("shape") == shape


def test_pso_get_shape():
    shape = ("Box", (Vec3(4, 1, 7),), TransformState.makePos((2, 4, 6)))
    obj = RBSO("rso")
    obj.set_shape(shape)
    assert obj.get_shape() == shape


def test_pso_create_shape():
    ts1 = TransformState.makePos((1, 3, 5))
    ts2 = TransformState.makePos((2, 4, 6))
    par2 = (Vec3(4, 1, 7),)
    obj0 = RBSO("0")
    obj1 = RBSO("1")
    obj2 = RBSO("2")
    obj0.set_shape("Box")
    obj1.set_shape(("Box", (), ts1))
    obj2.set_shape(("Box", par2, ts2))
    obj0.create_shape()
    obj1.create_shape()
    obj2.create_shape()
    sh0 = obj0.node().getShape(0)
    sh1 = obj1.node().getShape(0)
    sh2 = obj2.node().getShape(0)
    assert isinstance(sh0, BulletBoxShape)
    assert isinstance(sh1, BulletBoxShape)
    assert isinstance(sh2, BulletBoxShape)
    assert obj0.node().getShapeMat(0) == Mat4.identMat()
    assert obj1.node().getShapeMat(0) == ts1.getMat()
    assert obj2.node().getShapeMat(0) == ts2.getMat()
    assert sh0.getHalfExtentsWithMargin() == Vec3(0.5, 0.5, 0.5)
    assert sh1.getHalfExtentsWithMargin() == Vec3(0.5, 0.5, 0.5)
    assert sh2.getHalfExtentsWithMargin() == par2


def test_pso_delete_shape():
    obj = RBSO("rso")
    obj.set_shape("Box")
    obj.create_shape()
    obj.delete_shape()
    assert obj.node().getNumShapes() == 0


def test_bshapemanager():
    pass


def test_bshapemanager_read_node():
    pass


def test_bshapemanager_init():
    pass


## GSO
def test_gso_get_color():
    gso = GSO("foo")
    color = Vec4(0.2, 0.5, 0.8, 1.)
    gso.set_color(color)
    color2 = gso.get_color()
    assert color == color2


def test_gso_set_model():
    gso = GSO("foo")
    model = "smiley.egg"
    gso.set_model(model)
    assert model == gso.getPythonTag("model")


def test_gso_get_model():
    gso = GSO("foo")
    model = "smiley.egg"
    gso.set_model(model)
    model2 = gso.get_model()
    assert model == model2


def test_gso_create_model():
    gso = GSO("foo")
    model = "smiley.egg"
    gso.set_model(model)
    gso.create_model()
    nodes = gso.descendants()
    assert any(isinstance(n.node(), resource_types) for n in nodes)


def test_gso_delete_model():
    gso = GSO("foo")
    model = "smiley.egg"
    gso.set_model(model)
    gso.create_model()
    gso.delete_model()
    nodes = gso.descendants()
    assert not any(isinstance(n.node(), resource_types) for n in nodes)
