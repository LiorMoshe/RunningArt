import math
from decimal import Decimal
import navpy
import scipy
import pylab

"""
This class holds the implementation details of the rotation algorithm that is used for our figure after
each selection of a node.
It is used after each iteration of the algorithm to rotate the figure according to the difference in angles.
"""


#just for fun making further development easier and with joy
pi     = scipy.pi
dot    = scipy.dot
sin    = scipy.sin
cos    = scipy.cos
ar     = scipy.array
rand   = scipy.rand
arange = scipy.arange
plot   = pylab.plot
show   = pylab.show
axis   = pylab.axis
grid   = pylab.grid
title  = pylab.title
rad    = lambda ang: ang*pi/180                 #lovely lambda: degree to radian

def Rotate2D(pts,cnt,ang=pi/4):
    '''pts = {} Rotates points(nx2) about center cnt(2) by angle ang(1) in radian'''
    return dot(pts-cnt,ar([[cos(ang),sin(ang)],[-sin(ang),cos(ang)]]))+cnt

def rotate_path(path, theta):
    """
    Rotate the given path by theta.
    :param path:
    :param theta:
    :return:
    """
    pts = ar([[point[0], point[1]] for point in path])
    rotated = Rotate2D(pts, [path[0][0],path[0][1]], theta)
    updated_path = []
    for i in range(len(rotated)):
        updated_path.append([rotated[i][0], rotated[i][1], path[i][2]])
    return updated_path

def diff_based_rotation(seg_start, seg_end, node_start, node_end,segments):
    """
    Get the start and end points of both the segment and the chosen path
    and rotate the remaining segment points based on the difference in angles.
    This will cover up rotation mistakes that are made up by rotations oscilliations that are created
    by choices of the algorithm.
    Both points will be received in cartesian coordinates.
    :param seg_start: Starting point of the segment (cartesian).
    :param seg_end: Ending point of the segment (cartesian).
    :param node_start: Start point of the node (cartesian).
    :param node_end: End point of the node (cartesian).
    :param segments: List of our segments.
    :return:
    """
    if len(segments) == 0:
        return []

    seg_angle = math.atan2(seg_end[1] - seg_start[1], seg_end[0] - seg_start[0])
    node_angle = math.atan2(node_end[1] - node_start[1], node_end[0] - node_start[0])

    # Get the difference in angles and its direction and rotate the remaining points accordingly.
    diff = node_angle - seg_angle

    # Do not rotate if the difference is too wide, it will ruin the whole figure.
    if abs(diff) > math.pi/4:
        return segments
    points = []
    for i in range(len(segments)):
        seg = segments[i]
        start = list(navpy.lla2ecef(float(seg[0][0]), float(seg[0][1]), 0))
        points.append(start)
        if i == len(segments) - 1:
            end = list(navpy.lla2ecef(float(seg[1][0]), float(seg[1][1]), 0))
            points.append(end)

    rotated = rotate_path(points, theta=diff)
    new_segments = []
    for i in range(len(rotated)-1):
        ecef_point = navpy.ecef2lla([rotated[i][0], rotated[i][1], rotated[i][2]])
        next_ecef_point = navpy.ecef2lla([rotated[i+1][0], rotated[i+1][1], rotated[i+1][2]])
        new_segments.append(((Decimal(ecef_point[0]),Decimal(ecef_point[1])),
                               (Decimal(next_ecef_point[0]), Decimal(next_ecef_point[1]))))

    return new_segments

