

"""
Test the idea of finding the best fitting segments to the given location.
Currently we are pretty strict on the segments that we use based on the fonts that are
used in the computer.
Given the starting location we are going to iterate over the nodes
based on the angle of the figure. We keep our choices in a stack
so we can go backwards.
Everytime we get to a deadend we go backwards and we allow ourselves to "cut off" lengths of segments
in order to get a base "skeleton-like" segments that fit the given area.

When we know the given distance we can add/remove lengths of external edges.
"""
from segments.utils import get_closest_node, compute_latlong_angle
from collections import namedtuple
from copy import deepcopy
from geopy.distance import geodesic

AlgoState = namedtuple("AlgoState", "current_location constructed_segments remaining_segments leftover_distance")

def adjust_segments(segments, current_location, nodes_manager, threshold_angle=40):
    current_location = get_closest_node(current_location, nodes_manager.get_nodes())
    leftover_distance = 0
    nodes_ways = nodes_manager.get_nodes_ways()
    nodes_id_to_location = nodes_manager.get_nodes_map()


    # Stack of our past choices, allows us to go backwards.
    choices_stack = []
    constructed_segments=  []

    backtracked = False
    while len(segments) != 0:
        # Save current state in the stack.

        if not backtracked:
            choices_stack.append(AlgoState(current_location=deepcopy(current_location),
                                           constructed_segments=deepcopy(constructed_segments),
                                           remaining_segments=deepcopy(segments),
                                           leftover_distance=leftover_distance))
        backtracked = False


        curr_segment = segments.pop(0)

        # Get the angle and distance of the current segment.
        segment_angle = compute_latlong_angle(curr_segment[0][0],curr_segment[0][1],curr_segment[1][0],curr_segment[1][1])
        segment_distance = geodesic(curr_segment[0], curr_segment[1]).meters

        # Go over all the possible directions we can go towards.
        current_node_id = nodes_manager.get_node_id(current_location)

        # Choose the node with the closest angle.
        min_angle_diff = float('inf')
        best_neighbor = None
        for neighbor_id in nodes_ways[current_node_id]:
            neighbor_node = nodes_id_to_location[neighbor_id]
            nodes_angle = compute_latlong_angle(current_location[0], current_location[1], neighbor_node[0], neighbor_node[1])
            angle_diff = abs(nodes_angle - segment_angle)
            if angle_diff < min_angle_diff:
                min_angle_diff = angle_diff
                best_neighbor = neighbor_id


        if min_angle_diff < threshold_angle:
            # Save the current state before transitioning.


            nodes_distance = geodesic(current_location, best_neighbor).meters
            constructed_segments.append((current_location, best_neighbor))
            leftover_distance += nodes_distance - segment_distance
            current_location = best_neighbor
        else:
            # Backtrack, we are stuck.

            backtracked = True

    return constructed_segments