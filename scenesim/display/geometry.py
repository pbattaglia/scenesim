"""Tools associated with graphics geometry."""
import numpy as np


def zbuffer_to_z(zb, near, far):
    """Inputs Z-buffer image and returns each pixel's distance from the
    camera along the Z-axis.

    Args:
       zb (numpy.ndarray, 2D): Z-buffer image.
       near (float): Distance of near camera plane.
       far (float): Distance of far camera plane.

    Return:
       (numpy.ndarray, 2D): Z-distance of each pixel.

    """
    z = far * near / (far - zb * (far - near))
    return z


def img_to_d(xi, yi, zb, near, far):
    """Inputs image X, Y coordinates and Z-buffer image, and returns
    Euclidean distance from the camera's position to each point.

    Args:
       xi, yi (numpy.ndarray, 2D): X-, Y-coordinates of each pixel.
       zb (numpy.ndarray, 2D): Z-buffer image.
       near (float): Distance of near camera plane.
       far (float): Distance of far camera plane.

    Return:
       (numpy.ndarray, 2D): Euclidean distance of each pixel.

    """
    z = zbuffer_to_z(zb, near, far)
    phi = np.arctan2(np.sqrt(xi ** 2 + yi ** 2), near)
    d = z / np.cos(phi)
    return d


def img_to_xyz(xi, yi, zb, near, far):
    """Inputs image X, Y coordinates and Z-buffer image, and returns the
    pixels' X, Y, Z coordinates in 3D.

    Args:
       xi, yi (numpy.ndarray, 2D): X-, Y-coordinates of each pixel.
       zb (numpy.ndarray, 2D): Z-buffer image.
       near (float): Distance of near camera plane.
       far (float): Distance of far camera plane.

    Return:
       (numpy.ndarray, 3x2D): X-, Y-, Z-coordinates of each pixel.

    """
    z = zbuffer_to_z(zb, near, far)
    x = xi * z / near
    y = yi * z / near
    xyz = np.array((x, y, z))
    return xyz


def get_projection_mat(camera):
    """Projection matrix of camera.

    Args:
        camera (panda3d.core.NodePath): Camera NodePath.

    Return:
        (numpy.matrix, 4x4): Projection matrix (homogeneous).
    
    """
    lens = camera.node().getLens()
    frust_mat = np.matrix(lens.getProjectionMat())
    cam_mat = np.matrix(camera.getNetTransform().getMat())
    proj_mat = cam_mat.I * frust_mat
    return proj_mat


def extrude(point2d, proj_mat):
    """Compute the 3D inverse perspective projection of a 2D point.

    Args:
        point2d (numpy.ndarray, Nx2): Array of 2D points.
        proj_mat (numpy.matrix, 4x4): Projection matrix (homogeneous).

    Return:
        (numpy.ndarray, Nx3): Array of inverse projected 3D points.

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

    Args:
        point3d (numpy.ndarray, Nx{3,4}): Array of 3D (or 4D, if homogeneous) points.
        proj_mat (numpy.matrix, 4x4): Projection matrix (homogeneous).

    Return:
        (numpy.ndarray, Nx2): Array of inverse projected 2D points.
    
        (numpy.ndarray, N): Indicates points that are behind the camera.
    
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
    """ Compute point of intersection between a line and a plane.

    Args:
        line3 (numpy.ndarray, 2x3): 3D line defined by endpoints.
        point3 (numpy.ndarray, 3): 3D point on the plane.
        normal3 (numpy.ndarray, 3): 3D normal to the plane.

    Return:
        (numpy.ndarray, 3): 3D intersection point.
    
    """
    # Line ray
    ray = np.diff(line3, axis=0).ravel()
    # https://en.wikipedia.org/wiki/Line-plane_intersection
    d = np.dot(point3 - line3[0], normal3) / np.dot(ray, normal3)
    # Intersection point
    p3 = d * ray + line3[0]
    return p3
