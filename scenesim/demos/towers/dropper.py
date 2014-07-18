""" Tool for dropping blocks to create a tower."""
import numpy as np
import scipy.signal as sig
##
from ipdb import set_trace as BP


class DropperError(Exception):
    pass


class NoGoodLocationError(DropperError):
    pass


class Dropper(object):
    """ Creates 3D scenes containing blocks, that don't overlap and
    that have controllable geometric characteristics."""

    def __init__(self, base_scale, dims, RSO=0):
        self.RSO = RSO
        self.dims = np.array(dims, dtype="i")
        base_scale = np.array(base_scale, dtype="f8")
        self.step_size = base_scale / self.dims.astype("f8")
        self.xcoords = np.linspace(-base_scale[0] / 2., base_scale[0] / 2.,
                                   self.dims[0] + 1)[:-1]
        self.ycoords = np.linspace(-base_scale[1] / 2., base_scale[1] / 2.,
                                   self.dims[1] + 1)[:-1]
        # Compute region-sensitive weights.
        self.heightmap = None
        self.idmap = None
        self.reset()

    def reset(self):
        """ Resets the heightmap."""
        self.heightmap = np.zeros(self.dims)
        self.idmap = -np.ones(self.dims, dtype=np.int)

    def _weight_height(self, weight, hunq, h_ord):
        """ Computes the weighting of the indices based on
        different height weighting functions."""
        if not weight or hunq.size == 1:
            W = np.ones(h_ord.shape)
        else:
            # Compute the weights per height category
            if weight[0] == "floor":
                t = hunq == 0
            elif weight[0] == "stack":
                t = hunq > 0
            elif weight[0] == "height":
                t = hunq / float(hunq.max())
            W0 = t + (1.-t) * (1.-weight[1])
            # Apply weights across all heights in the category
            # (normalized for number of instances)
            W = np.empty(h_ord.shape)
            for u, w in zip(hunq, W0):
                ind = h_ord == u
                W[ind] = w / 1.  # ind.sum()
        return W

    # def weight_region(self, weight):
    #     """ Computes the weighting of the indices based on different
    #     region weighting functions."""
    #     if not weight:
    #         W = np.ones(self.dims)
    #     else:
    #         # Compute the weights via region function.
    #         W = weight(self.dims)
    #     # Normalize weights.
    #     W /= W.sum()
    #     return W

    def drop(self, scale, wr=1., id_=0):
        """ Drops a locally stable block.
        scale : (Nx3 np.ndarray) scale of the block to drop
        wr : region weighting function. Must have dimensions: self.dims
        id_ : (optional) block id, which prevents different ids from being
              placed in the same tower.
        """
        # epsilon (smallest value).
        eps = 0.01
        # Unique heights.
        hunq = np.round(np.unique(self.heightmap) / eps) * eps
        n_hunq = hunq.size
        # Object scale, quantized to grid.
        ### TODO
        # scale_int = np.ceil(np.array(scale[:2]) / self.step_size)
        ###
        scale_int = np.ceil(np.array(scale[:2]) / self.step_size + 1)
        ###
        # Kernel for object's footprint.
        kernel = np.ones(scale_int.astype("i"))
        # Kernel for keeping com over support.
        com_buffer = 4
        kernel_com = np.ones((com_buffer, com_buffer))
        kernel_com /= kernel_com.sum()
        conv = lambda X: (sig.convolve2d(X, kernel, mode="valid") > eps)
        conv_com = lambda X: (sig.convolve2d(
            X, kernel_com, mode="same", boundary="symm") > 1 - eps)
        conv_shape = self.dims - np.array(kernel.shape) + 1
        dx, dy = np.ceil(scale_int / 2.)
        conv_slice = (slice(dx, dx + conv_shape[0]),
                      slice(dy, dy + conv_shape[1]))
        # Init good locations.
        open_ind = np.zeros(conv_shape, dtype="b")
        ## Iterate over heights in the heightmap.
        for h in hunq:
            # Lower height locations.
            lower_ind = self.heightmap - eps < h
            # Same height locations.
            same_ind = np.abs(self.heightmap - h) < eps
            # Higher locations.
            higher_ind = ~(lower_ind | same_ind)
            # Indices of filled locations.
            filled_ind = conv(higher_ind.astype("f8"))
            # Supportable locations.
            support_ind = conv_com(same_ind)
            open_ind |= ~filled_ind & support_ind[conv_slice[0], conv_slice[1]]
        good_ind = open_ind.copy()
        if True:
            ## Disqualify competitors' heightmaps.
            competitor_ind = (self.idmap >= 0) & (self.idmap != id_)
            competitor_ind = conv(competitor_ind.astype("f8"))
            good_ind &= ~competitor_ind
        # Height order.
        h_ord = np.searchsorted(hunq, self.heightmap)
        # Compute height-sensitive weights.
        wh = self._weight_height(None, np.arange(n_hunq), h_ord)
        # Indices of good locations.
        idx = np.flatnonzero(good_ind)
        w = wh * wr
        p = w[conv_slice[0], conv_slice[1]].flat[idx]
        if not np.any(p > 0.):
            # No good locations available.
            return None
        ## There are good locations available. Choose one at random.
        s = float(p.sum())
        p = np.ones_like(p) / float(p.size) if s <= eps else p / s
        r = self.RSO.choice(idx, p=p)
        # Get the corresponding x,y,z coordinates.
        sub = np.array(np.unravel_index(r, good_ind.shape))
        x = self.xcoords[sub[0] + dx]
        y = self.ycoords[sub[1] + dy]
        h = self.heightmap[tuple(sub + np.array((dx, dy)))]
        z = h + scale[2] / 2.
        # Update self.heightmap and self.idmap.
        i0 = max(sub[0], 0)
        i1 = min(sub[0] + scale_int[0], self.dims[0])
        j0 = max(sub[1], 0)
        j1 = min(sub[1] + scale_int[1], self.dims[1])
        # from matplotlib import pyplot as plt
        # plt.figure(1)
        # plt.clf()
        # plt.subplot(1, 2, 1)
        # hm = self.heightmap.copy()
        # plt.imshow(hm, interpolation="None")
        # plt.draw()
        self.heightmap[i0:i1, j0:j1] = h + scale[2]
        self.idmap[i0:i1, j0:j1] = id_
        # plt.subplot(1, 2, 2)
        # plt.imshow(self.heightmap, interpolation="None")
        # plt.draw()
        # plt.show()
        # BP()
        return x, y, z
