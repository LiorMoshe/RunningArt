

"""
This file contains methods that we use to choose the best route among all the possible routes that we
return from our algorithm.
"""
from segments.utils import compute_latlong_angle
import math
import operator


def count_turns_in_path(path, threshold=60):
    turns = 1

    prev_angle = None
    for i in range(len(path) - 1):
        node = path[i]
        next_node = path[i+1]

        curr_angle = compute_latlong_angle(node[0], node[1], next_node[0], next_node[1])

        if prev_angle is None:
            prev_angle = curr_angle

        angle_diff = min(abs(prev_angle - curr_angle), abs(360 - abs(prev_angle - curr_angle)))
        if angle_diff > threshold:
            prev_angle = curr_angle
            turns += 1

    print("Total Turns: {0}".format(turns))
    return turns


def turn_based_arbitrator(possible_routes, movements, node_id_to_location):
    """
    Extract from a set of routes the ones that have the same number of turns as the
    given set of movements.
    :param possible_routes:
    :param movements:
    :return:
    """
    routes = []
    required_turns = len(movements)
    for route in possible_routes:
        path = route.get_full_location_path(node_id_to_location)
        if count_turns_in_path(path) == required_turns:
            routes.append(route)
    return routes

def compare_movements(first_shape, second_shape):
    """
    Compare the movements that construct the same shape.
    Assumes that both shapes have the same number of movements.
    :param first_shape:
    :param second_shape:
    :return:
    """
    if len(first_shape) != len(second_shape):
        raise Exception("Expecting both shapes to have the same number of movements, First: {0} Second: {1}"
                        .format(len(first_shape), len(second_shape)))

    total = 0.0
    num_movements = len(first_shape)
    for i in range(num_movements):
        total += math.sin(min(math.pi/2,abs(first_shape[i].angle - second_shape[i].angle) * math.pi/180)) * \
            abs(first_shape[i].magnitude - second_shape[i].magnitude)
    return total


def movement_based_arbitrator(possible_routes, movements, node_id_to_location, include_breaks=False):
    """
    Compare the set of movements of each  possible route to the original movements.
    Currently we act as if we expect the result to have the exact same number of movements as the input.
    In the future it is going to change since we are going to approximate one long straight movement
    by taking several small ones at opposite angles (zig-zagging). TODO- Set this up with the include_breaks variable.


    :param possible_routes:
    :param movements:
    :param node_id_to_location:
    :param include_breaks:
    :return:
    """
    idx_to_score = {}
    for idx, route in enumerate(possible_routes):
        route_movements = route.get_movements(node_id_to_location)
        if len(route_movements) != len(movements) and not include_breaks:
            idx_to_score[idx] = float('inf')
            continue

        # Compare the given movements to the input movement (this is purely a speculation).
        idx_to_score[idx] = compare_movements(route_movements, movements)

    print("Total scores: ", idx_to_score)
    best_idx, best_score = min(idx_to_score.items(),key=operator.itemgetter(1))
    return possible_routes[best_idx]
