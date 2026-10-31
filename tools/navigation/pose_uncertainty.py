"""Planar pose covariance validation and route-frame projections. No ROS."""
import math
import numpy as np


def planar_covariance(values):
    matrix=np.asarray(values,dtype=float)
    if matrix.size==36:
        matrix=matrix.reshape(6,6)[np.ix_([0,1,5],[0,1,5])]
    if matrix.shape!=(3,3) or not np.isfinite(matrix).all():
        raise ValueError('Invalid planar pose covariance')
    if not np.allclose(matrix,matrix.T,atol=1e-9,rtol=0.):
        raise ValueError('Asymmetric planar pose covariance')
    matrix=.5*(matrix+matrix.T)
    if np.any(np.diag(matrix)<0):
        raise ValueError('Negative planar pose covariance diagonal')
    if np.min(np.linalg.eigvalsh(matrix)) < -1e-10:
        raise ValueError('Indefinite planar pose covariance')
    return matrix


def project_uncertainty(matrix, route_yaw):
    if not math.isfinite(route_yaw):
        raise ValueError('Invalid route heading')
    matrix=planar_covariance(matrix)
    tangent=np.array([math.cos(route_yaw),math.sin(route_yaw)])
    normal=np.array([-tangent[1],tangent[0]])
    planar=matrix[:2,:2]
    across=math.sqrt(max(0.,float(normal@planar@normal)))
    along=math.sqrt(max(0.,float(tangent@planar@tangent)))
    major=math.sqrt(max(0.,float(np.linalg.eigvalsh(planar)[-1])))
    yaw=math.sqrt(max(0.,float(matrix[2,2])))
    return across,along,major,yaw
