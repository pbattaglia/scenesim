#!/usr/bin/python
"""Generate tower scenes."""
import sys
##
import numpy as np
from panda3d.core import Point3, Vec3, Vec4, Quat
from path import path
from scenesim.objects import GSO, RBSO, SSO
##
from dropper import Dropper
##
from ipdb import set_trace as BP


PLANE = True
# BLOCK_SIZE_RNG = np.array((0.8, 1.2))
BLOCK_SIZE_RNG = np.array((1., 1.))
BLOCK_AR = 0.2 * np.array(((3., 1., 1.),
                           (1., 1., 3.),
                           (1., 3., 1.),
                           #
                           (2, 1., 1.),
                           (1., 1., 2),
                           #
                           (1., 1., 1.)))
RESOLUTION = 0.02
if PLANE:
    TABLE_SZ = np.array((0.8, 0.2))
    DIMS = np.ceil(TABLE_SZ / RESOLUTION).astype('i')
    DIMS[1] = 2
    BLOCK_PROB = np.array((1., 1, 0, 0, 0, 0))
else:
    TABLE_SZ = np.array((0.6, 0.6))
    DIMS = np.ceil(TABLE_SZ / RESOLUTION).astype('i')
    BLOCK_PROB = np.array((1., 1, 1, 0, 0, 0))
# BLOCK_PROB = np.array((1., 1, 1, 1, 2))
BLOCK_PROB /= BLOCK_PROB.sum()
COLOR = {
    'red': (1., 0.2, 0.2, 1.),
    # 'pink': (1., 0.4, 0.7, 1.),
    # 'orange': (1., 0.65, 0., 1.),
    'yellow': (1., 1., 0.3, 1.),
    'green': (0.3, 0.7, 0.3, 1.),
    'cyan': (0., 1., 1., 1.),
    'blue': (0.4, 0.4, 1., 1.),
    'purple': (0.5, 0., 0.5, 1.),
    # 'white': (1., 1., 1., 1.),
    'gray': (0.5, 0.5, 0.5, 1.),
    # 'black': (0., 0., 0., 1.),
    'brown': (0.45, 0.25, 0.15, 1.)
    }
LUMINANCE_RNG = np.array((0.3, 0.95))


def init_RSO(RSO):
    """ Takes either an existing mtrand.RandomState object, or a seed,
    and returns an mtrand.RandomState object."""
    # Random state
    if not isinstance(RSO, np.random.mtrand.RandomState):
        # RSO is a seed
        RSO = np.random.RandomState(seed=RSO)
    return RSO


def make_block_shapes(n=1, RSO=0):
    # Make block scales.
    ar = RSO.choice(BLOCK_AR.shape[0], size=n, p=BLOCK_PROB)
    sr = RSO.rand(n, 1) * np.diff(BLOCK_SIZE_RNG) + BLOCK_SIZE_RNG[0]
    sr = np.hstack((sr, np.ones_like(sr), sr))
    # Scales
    scales = BLOCK_AR[ar] * sr
    # floor 'scales' to RESOLUTION
    scales -= scales % RESOLUTION
    return scales


def find_locations(scales, RSO=0):
    def _drop(dm, scale, X):
        xyz = dm.drop(scale)
        f_success = xyz is not None
        if f_success:
            X.append(np.array(xyz))
        return f_success
    # Parameters
    base_scale = TABLE_SZ.copy()
    base_scale[1] *= 2
    n_fail = 20
    count = 0
    f_success = False
    dm = Dropper(base_scale, DIMS, RSO=RSO)
    while not f_success:
        if count >= n_fail:
            return None
        if count > 0:
            print('No good location. Retrying x %i.' % count)
        dm.reset()
        f_success = True
        X = []
        for scale in scales:
            f_success &= _drop(dm, scale, X)
            if not f_success:
                break
        count += 1
    return X


def create_block(pos, scale, color, tag):
    """ Creates a single block."""
    # Physics state.
    pstate = {'name': tag,
              'pos': Vec3(*pos),
              'scale': Vec3(*scale),
              'shape': 'Box',
              'friction': 1.,
              'restitution': 0.,
              'mass': 1.}
    # Graphics state.
    gstate = {'name': tag,
              'model': 'block.egg',
              'color': Vec4(*color)}
    # Make the object.
    pso = RBSO(props=pstate)
    gso = GSO(props=gstate)
    gso.reparent_to(pso)
    return pso


def make_blocks(block_data, RSO=0):
    n = block_data['n']
    S = make_block_shapes(n=n, RSO=RSO)
    X = find_locations(S, RSO=RSO)
    if X is None:
        return None, None
    colors = COLOR.values()
    RSO.shuffle(colors)
    psos = []
    for i, (x, s, color) in enumerate(zip(X, S, colors)):
        tag = 'block_%02i' % i
        pso = create_block(x, s, color, tag)
        psos.append(pso)
    return psos


def assemble_scene(name, psos):
    scene = SSO(props={'name': name})
    for pso in psos:
        pso.reparent_to(scene)
    return scene


def make_scenes(RSO=0):
    """Make the scenes."""
    N = [50, 50, 50]  # 3 groups of scenes, 50 scenes each.
    block_data = [{'n': 4}, {'n': 6}, {'n': 8}]  # Blocks per scene.
    n_scenes = sum(N)
    scenes = []
    for i, (n, bd) in enumerate(zip(N, block_data)):
        for iscene in xrange(n):
            psos = []
            while not psos:
                psos = make_blocks(bd, RSO=RSO)
            j = sum(N[:i]) + iscene
            name = 'tower_%06i_%02i' % (j, bd['n'])
            scene = assemble_scene(name, psos)
            scenes.append(scene)
            print('%i / %i' % (j, n_scenes))
    return scenes


def make_ground():
    """ Makes ground."""
    color = Vec4(0.1, 0.1, 0.1, 1.0)
    # Dimensions.
    dxy = Point3(0., 0., 0.)
    scl_ground = Vec3(8., 8., 2.)
    # Create ground physics node.
    bstate = {
        'name': 'ground',
        'scale': scl_ground,
        'pos': Point3(0., 0., -scl_ground[2] / 2.) + dxy,
        'quat': Quat(1., 0., 0., 0.),
        'shape': 'Box',
        'friction': 1.,
        'restitution': 0.,
        'mass': 0.
        }
    pso = RBSO(props=bstate)
    # Create ground graphics node.
    gstate = {'name': pso.get_name(),
              'model': 'cylinderZ.egg',
              'color': color}
    gso = GSO(props=gstate)
    gso.reparentTo(pso)
    return pso


def save_node(node, dir_pth, filename=None, f_force=False):
    """Save the scene stimuli to disk."""
    if not filename:
        filename = '%s.sso' % node.get_name()
    pth = path.joinpath(dir_pth, filename)
    if not f_force and pth.isfile():
        print('File exists, skipping: %s' % pth)
    else:
        if not pth.dirname().exists():
            pth.dirname().makedirs()
        with pth.open('w') as fid:
            node.save_tree(fid)


def run(stim_dir, RSO=0):
    # Make ground
    ground = make_ground()
    # Save ground
    save_node(ground, dir_pth, f_force=True)
    # Make the scene stimuli.
    scenes = make_scenes(RSO=RSO)
    # Save towers.
    for iscene, scene in enumerate(scenes):
        filename = '%s.sso' % scene.get_name()
        save_node(scene, dir_pth, filename=filename, f_force=True)
    print('Complete.')
    return ground, scenes


if __name__ == '__main__':
    seed = 3
    RSO = init_RSO(seed)
    args = sys.argv[1:]
    try:
        dir_pth = path(args[0])
    except IndexError:
        dir_pth = path('tower_ssos')
    ground, scenes = run(dir_pth, RSO=RSO)
