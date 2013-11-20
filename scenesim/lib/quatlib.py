"""
Quaternion tools
"""
from itertools import imap, product
import numpy as np


_mtx = np.array([[[1, 0, 0, 0],
                  [0, -1, 0, 0],
                  [0, 0, -1, 0],
                  [0, 0, 0, -1]],
                 [[0, 1, 0, 0],
                  [1, 0, 0, 0],
                  [0, 0, 0, 1],
                  [0, 0, -1, 0]],
                 [[0, 0, 1, 0],
                  [0, 0, 0, -1],
                  [1, 0, 0, 0],
                  [0, 1, 0, 0]],
                 [[0, 0, 0, 1],
                  [0, 0, 1, 0],
                  [0, -1, 0, 0],
                  [1, 0, 0, 0]]], dtype="f8")


def as_matrix(q):
    """ Returns matrix representation of quaternion `q`. Quaternion
    vector is on right-most axis."""
    q = np.array(q)
    if q.ndim > 1:
        q = q[..., None]
    m = np.dot(_mtx, q)
    if q.ndim > 2:
        # Move left-most `q` dimensions back to the left.
        m = m[..., 0].transpose(range(2, q.ndim) + [0, 1])
    return m


def almost_equal(x, y, axis=-1, tol=1e-6):
    """
    Tests whether x and y are within tol of each other
    """
    test = np.abs(x - y) < tol
    if axis > test.ndim - 1 or test.ndim + axis < 0:
        fequal = test
    else:
        fequal = np.all(test, axis=axis)
    return fequal


def conj(q0, axis=-1):
    """
    Returns complex conjugate (inverse) of quaternion (numpy.ndarray)
    """
    q0 = np.array(q0)
    if q0.shape[axis] != 4:
        raise ValueError("q0's axis dimension must be length 4")
    # Need to resign the i,j,k complex components
    signshape = np.ones(q0.ndim)
    signshape[axis] = 4
    sign = np.array([1, -1, -1, -1]).reshape(signshape, order='F')
    q = q0 * sign
    return q


# def prod(q0, q1):
#     """ Returns quaternion product between `q0` and `q1`. Quaternion
#     vectors are on right-most axes."""
#     p = np.dot(as_matrix(q0), as_matrix(q1)[..., [0]])[..., 0]
#     if q1.ndim > 1:
#         m = q0.ndim
#         n = m + q1.ndim - 1
#         p = p.transpose(range(m - 1) + range(m, n) + [m - 1])
#     return p


def prod(q0, q1):
    """ Returns quaternion product between `q0` and `q1`. Quaternion
    vectors are on right-most axes."""
    m0 = as_matrix(q0)
    m1 = as_matrix(q1)[..., 0]
    sh = m0.shape[:-2]
    if sh != m1.shape[:-1]:
        raise ValueError("Input dimensions do not match.")
    if len(sh) > 0:
        p = np.empty((sh + (4,)))
        for subs in product(*imap(xrange, sh)):
            p[subs] = np.dot(m0[subs], m1[subs])
    else:
        p = np.dot(m0, m1)
    # p = np.dot(as_matrix(q0), as_matrix(q1)[..., [0]])[..., 0]
    # if q1.ndim > 1:
    #     m = q0.ndim
    #     n = m + q1.ndim - 1
    #     p = p.transpose(range(m - 1) + range(m, n) + [m - 1])
    return p


def length2(q0, axis=-1):
    """
    Computes square of quaternion length
    """
    if q0.shape[axis] != 4:
        raise ValueError("q0's axis dimension must be length 4")
    S = np.sum(q0 ** 2, axis)
    if q0.ndim > 1:
        # Don't L2 lose a dimension
        sl = [slice(None)] * q0.ndim
        sl[axis] = None
        L2 = S[sl]
    else:
        L2 = S
    return L2


def norm(q0, axis=-1):
    """
    Computes quaternion norm
    """
    if q0.shape[axis] != 4:
        raise ValueError("q0's axis dimension must be length 4")
    L = length2(q0, axis) ** 0.5
    return L


def is_norm(q0, axis=-1, tol=1e-6):
    """
    Checks whether quaternion is normalized
    """
    if q0.shape[axis] != 4:
        raise ValueError("q0's axis dimension must be length 4")
    L2 = length2(q0, axis)
    fnorm = almost_equal(L2, 1., axis=axis, tol=tol)
    return fnorm


isNorm = is_norm


def normalize_in_place(q, axis=-1):
    """
    Normalize quaternion to 1, in place
    """
    q /= norm(q, axis=axis)


def normalize(q0, axis=-1):
    """
    Normalize quaternion to 1
    """
    q = q0.copy()
    normalize_in_place(q, axis=axis)
    return q


def pow(q, p, axis=-1):
    """ Raise quaternion `q` to power `p`."""
    ang = np.arccos(q.take([0], axis=axis))
    v = q.take(xrange(1, q.shape[axis]), axis=axis) / np.sin(ang)
    v[np.isnan(v)] = 0.
    pang = p * ang
    qp = np.concatenate((np.cos(pang), v * np.sin(pang)), axis=axis)
    return qp


def diff(q0, q1, axis=-1):
    """ Return difference between `q1` and `q0`. For rotations, this
    amounts to inverting the rotation represented by `q0` and then
    rotating to `q1`."""
    return prod(conj(q0, axis=axis), q1)


def angle(q, axis=-1):
    """
    Computes angle (radians) of unit quaternion
    """
    q = np.array(q)
    if q.shape[axis] != 4:
        raise ValueError("q0's axis dimension must be length 4")
    tol = 1e-6
    if not np.all(is_norm(q, axis=axis, tol=tol)):
        raise ValueError("q0 is not normalized -- angle is meaningless")
    ang = 2. * np.arccos(q.take([0], axis=axis))[..., 0]
    return ang


def slerp(q0, q1, t):
    """
    Return spherically-interpolated quaternion
    q0, q1 : 2 quaternions to interpolate between
    t : interpolation factor
    """
    qs = prod(q0, pow(diff(q0, q1), t))
    # ang = angle(diff(q0, q1), axis=-1) / 2.
    # qs0 = (q0 * np.sin((1 - t) * ang) + q1 * np.sin(t * ang)) / np.sin(ang)
    return qs  # qs0


def euler(quat, axis=-1):
    """
    Computes Euler angles (radians) of unit quaternion.
    http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    """
    if quat.shape[axis] != 4:
        raise ValueError("quat's axis dimension must be length 4")
    tol = 1e-6
    if not np.all(is_norm(quat, axis=axis, tol=tol)):
        raise ValueError("quat is not normalized -- angle is meaningless")
    q0 = quat.take([0], axis=axis)
    q1 = quat.take([1], axis=axis)
    q2 = quat.take([2], axis=axis)
    q3 = quat.take([3], axis=axis)
    phi = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2))
    theta = np.arcsin(2 * (q0 * q2 - q3 * q1))
    psi = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 ** 2 + q3 ** 2))
    angs = np.concatenate((phi, theta, psi), axis=axis)
    return angs


def quat2mat(Q):
    """Return homogeneous rotation matrix from quaternion.

    (adapted from http://www.lfd.uci.edu/~gohlke/code/transformations.py.html)

    >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    >>> numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    >>> M = quaternion_matrix([1, 0, 0, 0])
    >>> numpy.allclose(M, numpy.identity(4))
    True
    >>> M = quaternion_matrix([0, 1, 0, 0])
    >>> numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
    True

    """

    _EPS = 0.00001
    q = np.array(Q, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        M = np.identity(4)
    else:
        q *= np.sqrt(2.0 / n)
        q = np.outer(q, q)
        M = np.array([
            [1. - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0], 0.],
            [q[1, 2] + q[3, 0], 1. - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0], 0.],
            [q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1. - q[1, 1] - q[2, 2], 0.],
            [0.0, 0.0, 0.0, 1.0]])
    return M


# def prod0(q0, q1, axis=0):
#     """
#     Returns quaternion product of two quat matrices (numpy.ndarray)
#     modified from (note their point about matrix transpose -- the
#     website has the matrix wrong, so just perform the transpose as
#     they say):
#     http://www.genesis3d.com/~kdtop/Quaternions-UsingToRepresentRotation.htm
#     """
#     if q0.shape[axis] != 4:
#         raise ValueError("q0 and q1's axis dimension must be length 4")
#     ndim1 = q1.ndim
#     # Matrix formatted transpose order
#     neword = range(ndim1)
#     neword.append(neword.pop(axis))
#     # Transpose order to reverse back to original shape
#     revord = range(ndim1)
#     revord.insert(axis, revord[-1])
#     revord.pop()
#     # Change order and shape of dimensions
#     nq1 = np.transpose(q1, neword)
#     oldsh = nq1.shape  # we'll need this later to return to the original shape
#     nq1 = nq1.reshape((-1, 4), order='F')
#     # These are the sign and element rearrangements/resigns necessary
#     # to perform the quaternion multiplication as a matrix multiply
#     sign = [np.array([[1, 0, 0, 0],
#                       [0, -1, 0, 0],
#                       [0, 0, -1, 0],
#                       [0, 0, 0, -1]]),
#             np.array([[0, 1, 0, 0],
#                       [1, 0, 0, 0],
#                       [0, 0, 0, 1],
#                       [0, 0, -1, 0]]),
#             np.array([[0, 0, 1, 0],
#                       [0, 0, 0, -1],
#                       [1, 0, 0, 0],
#                       [0, 1, 0, 0]]),
#             np.array([[0, 0, 0, 1],
#                       [0, 0, 1, 0],
#                       [0, -1, 0, 0],
#                       [1, 0, 0, 0]])]
#     # List of rearranged/resigned q1s: do the matrix multiply, then
#     # reshape, then transpose back to original shape
#     lrq1 = [np.transpose(np.reshape(np.dot(nq1, s), oldsh, order='F'), revord)
#             for s in sign]
#     del(nq1) # free up memory
#     # This allows custom slicing to set values of qprod array
#     slices = [slice(None)] * ndim1
#     # Calculate final product's shape
#     shape = np.max(np.vstack((q0.shape, q1.shape)), axis=0)
#     # This shape is needed for the single line
#     shapeq = shape.copy()
#     shapeq[axis] = 1
#     # Allocate
#     qprod = np.empty(shape)
#     # Iterate through each term of lrq1 and do the dot product with q0
#     for rq1, i in zip(lrq1, range(4)):
#         # Make this term's slice
#         slc = slices[:]
#         slc[axis] = [i]
#         # Compute final dot product
#         qprod[slc] = np.sum(q0 * rq1, axis).reshape(shapeq, order='F')
#     return qprod





# THIS IS JUST SCRAP CODE TO VERIFY QUATERNION FUNCTIONS AGAINST PANDA3D'S QUAT OBJECT...

# from pandac.core import Quat

# q0 = np.random.rand(3, 4, 2) * 2 - 1
# q1 = np.random.rand(3, 4, 2) * 2 - 1
# dq = quatprod(q0, q1, axis=1)

# QQ=[]
# QC=[]
# for i in range(3):
#     for k in range(2):
#         Q0=Quat(Vec4(*q0[i,:,k]))
#         Q1=Quat(Vec4(*q1[i,:,k]))
#         QQ.append(Q0 * Q1)
#         QC.append([Q0.conjugate(), Q1.conjugate()])

# from pprint import PrettyPrinter
# pp = PrettyPrinter()

# pp.pprint(QQ)
# pp.pprint(dq)

# pp.pprint(QC)
# pp.pprint([quatconj(q0, axis=1), quatconj(q1, axis=1)])
#################

# def QQ(*args):
#     if len(args) == 4:
#         Q = Quat(*args)
#     elif len(args) == 1:
#         Q = Quat(*args[0])
#     else:
#         raise ValueError("QQ args must be 4 scalars, or 1 4-tuple")
#     return Q

# pd = pardisp.transpose((0, 1, 2, 3, 4, 6, 5)).reshape((-1, 7), order='F')[:, 3:]
# cd = cpodisp.transpose((0, 1, 2, 3, 4, 6, 5)).reshape((-1, 7), order='F')[:, 3:]

# qpd = pd.copy()
# q.normalizeInPlace(qpd, -1)
# #qcd = q.norm(q.normalize(cd, -1), -1)

# for i in xrange(4963): #pd.shape[0]):
#     qipd = QQ(pd[i, :])
#     qipd.normalize()
#     a = qipd.getAngleRad()
#     #qicd = QQ(cd[i, :])
#     #qicd.normalize()
#     print a - q.angle(qpd[i, :])
#     #print qicd - QQ(qcd[i, :]), '\n'
