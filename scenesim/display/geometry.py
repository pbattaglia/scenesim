""" Various graphics geometry manipulation functions."""
import numpy as np


def zbuffer_to_z(zb, near, far):
    """ Inputs Z-buffer image and returns each pixel's distance from
    the camera along the z-axis."""
    z = far * near / (far - zb * (far - near))
    return z


def img_to_d(xi, yi, zb, near, far):
    """ Inputs image x, y coordinates and Z-buffer image, and returns
    distance from the camera's position to eachpoint."""
    z = zbuffer_to_z(zb, near, far)
    phi = np.arctan2(np.sqrt(xi ** 2 + yi ** 2), near)
    d = z * np.cos(phi)
    return d


def img_to_xyz(xi, yi, zb, near, far):
    """ Inputs image x, y coordinates and Z-buffer image, and returns
    the pixels' x, y, z coordinates in 3D."""
    z = zbuffer_to_z(zb, near, far)
    x = xi * z / near
    y = yi * z / near
    xyz = np.array((x, y, z))
    return xyz


def get_projection_mat(camera):
    """ Projection matrix of camera."""
    lens = camera.node().getLens()
    frust_mat = np.matrix(lens.getProjectionMat())
    cam_mat = np.matrix(camera.getNetTransform().getMat())
    proj_mat = cam_mat.I * frust_mat
    return proj_mat


def extrude(point2d, proj_mat):
    """ Compute the 3D inverse perspective projection of a 2D point.
    point2d : Nx{2} set of 2D points
    proj_mat : 4x4 camera projection matrix
    #
    point3d : Nx3 inverse projected 3D points
    """
    # Inverse projection matrix
    proj_mat_inv = np.linalg.inv(proj_mat)
    # calculate the near and far points
    hp2 = np.array(((point2d[0], point2d[1], -1., 1.),
                    (point2d[0], point2d[1], 1., 1.)))
    hp3 = np.dot(hp2, proj_mat_inv)
    scale = hp3[:, [3]].copy()
    thresh = 0.00001
    scale[(scale > 0) & (scale < thresh)] = thresh
    scale[(scale < 0) & (scale > -thresh)] = -thresh
    point3d = np.array(hp3[:, :3] / scale)
    return point3d


def project(point3d, proj_mat):
    """ Compute the 2D perspective projection of 3D point(s).
    point3d : Nx{3,4} set of 3D points (if Nx4, they are homogeneous coords)
    proj_mat : 4x4 camera projection matrix
    #
    point2d : Nx2 projected 2D points
    f_behind : indicates points that are behind the camera
    """
    # Cast to np.array
    point3d = np.array(point3d)
    if point3d.ndim == 1:
        # Add dimension in front, if necessary
        point3d = point3d[None, :]
    # Make homogeneous coordinates from point3d
    d1len = point3d.shape[1]
    if d1len == 3:
        hp3 = np.hstack((point3d, np.ones(point3d.shape[0])[:, None]))
    elif d1len == 4:
        hp3 = point3d
    else:
        raise ValueError("point3d must be either Nx{3,4}, but it is %i" %
                         d1len)
    # Compute the linear portion of the projection
    hp2 = np.dot(hp3, proj_mat)
    # Compute z-scaling
    point2d = np.array(hp2[:, :2] / hp2[:, [3]])
    f_behind = hp2[:, 2] < 0
    return point2d, f_behind


def plane_intersection(line3, point3, normal3):
    """ Compute point of intersection between a line and a plane."""
    # Line ray
    ray = np.diff(line3, axis=0).ravel()
    # https://en.wikipedia.org/wiki/Line-plane_intersection
    d = np.dot(point3 - line3[0], normal3) / np.dot(ray, normal3)
    # Intersection point
    p3 = d * ray + line3[0]
    return p3
