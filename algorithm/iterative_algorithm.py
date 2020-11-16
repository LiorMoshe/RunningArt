

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


import logging
from _decimal import Decimal

from segments.utils import get_closest_node, compute_latlong_angle
from collections import namedtuple
from copy import deepcopy
from geopy.distance import geodesic
from algorithm.arbitrator import *

AlgoState = namedtuple("AlgoState", "current_location constructed_segments remaining_segments leftover_distance")

# Angle is in degrees
Movement = namedtuple("Movement", "angle magnitude")

Action = namedtuple("Action", "movement_idx chosen_node dist prev_node")


def setup_logger(name, log_file, level=logging.INFO):
    """To setup as many loggers as you want"""
    logging.basicConfig(
        filename='logger.log',
        level=logging.DEBUG,
        format='%(asctime)s.%(msecs)03d %(levelname)s %(module)s - %(funcName)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
    )

    handler = logging.FileHandler(log_file,mode='w')
    # handler.setFormatter(formatter)

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)

logger = setup_logger('first_logger', 'logger.log')
logger = logging.getLogger('first_logger')

class PossibleRoute(object):

    def __init__(self, movements, node_id):
        self.nodes_per_movement = {}
        self.movements = movements
        self.current_movement = movements[0]
        self.current_node_id = node_id

        self.movement_idx = 0
        self.org_movement_dist = {i:movements[i].magnitude for i in range(len(movements))}
        self.dist_per_movement = {i:movements[i].magnitude for i in range(len(movements))}



        self.done = False

        self.expansion_mode = False


        self.actions = []
        self.path = []


    def finished_movement(self):
        self.expansion_mode = False

        # If we finished last movement - figure is done.
        if self.is_on_last_movement():
            self.done = True
        else:
            # Jump to the next movement.
            self.advance_movement()

    def is_valid(self):
        """
        Tests whether we should continue searching the path in this route.
        :return:
        """
        return (not self.expansion_mode) or (self.expansion_mode and
                                             -self.dist_per_movement[self.movement_idx] < self.org_movement_dist[self.movement_idx])

    def add_node_for_movement(self, node_id, dist):
        if self.movement_idx not in self.nodes_per_movement:
            self.nodes_per_movement[self.movement_idx] = []


        self.nodes_per_movement[self.movement_idx].append(node_id)

        # Add the accumulated distance. Jump to next movement if the distance is covered.
        self.dist_per_movement[self.movement_idx] -= dist

        self.path.append(node_id)

        self.actions.append(Action(self.movement_idx, node_id, dist, self.current_node_id))

        logging.info("Adding node {0} for movement {1}, remaining dist {2}".format(node_id, self.movement_idx,
                                                                                   self.dist_per_movement[self.movement_idx]))

        self.current_node_id = node_id
        if not self.expansion_mode and self.dist_per_movement[self.movement_idx] <= 0:
            self.finished_movement()

    def extend_movement(self):
        """
        Extend the movement.
        TODO-Extending up to two times the original distance, maybe will be changed in future.
        :param movement_idx:
        :return:
        """
        self.expansion_mode = True

    def advance_movement(self):
        self.movement_idx += 1
        self.current_movement = self.movements[self.movement_idx]

    def set_previous_movement(self):
        if self.movement_idx == 0:
            raise Exception("Can't go back. Current movement is already 0.")
        self.movement_idx -= 1
        self.current_movement = self.movements[self.movement_idx]

    def is_on_last_movement(self):
        return self.movement_idx == len(self.movements) - 1

    def get_next_movement(self):
        if self.is_on_last_movement():
            raise Exception("Can't get next movement, route is already at the last movement")
        return self.movements[self.movement_idx + 1]

    def num_nodes_of_movement(self, movement_idx):
        if movement_idx in self.nodes_per_movement:
            return len(self.nodes_per_movement[movement_idx])
        return 0

    def backtrack(self):
        """
        Go one node backwards on our path. Revert based on the actions that we saved.
        :return:
        """
        if len(self.actions) == 0:
            raise Exception("Cannot backtrack when there are no saved actions.")
        last_action = self.actions.pop(len(self.actions) - 1)
        self.dist_per_movement[last_action.movement_idx] += last_action.dist
        self.nodes_per_movement[last_action.movement_idx].remove(last_action.chosen_node)
        self.current_node_id = last_action.prev_node
        self.path.pop(len(self.path) - 1)
        self.movement_idx = last_action.movement_idx

    def get_full_location_path(self, node_id_to_location):
        location_path = []
        for node_id in self.path:
            location = node_id_to_location[node_id]
            location_path.append((float(location[0]), float(location[1])))
        return location_path

    def get_movements(self, node_id_to_location):
        """
        Convert the given route to a set of movements
        :return:
        """
        location_path = self.get_full_location_path(node_id_to_location)
        # Structure the path as segments.
        segments = []
        for i in range(len(location_path) - 1):
            segments.append((location_path[i],location_path[i+1]))
        return simplify_segments(segments)

    def num_actions(self):
        return len(self.actions)

    def __deepcopy__(self, memodict={}):
        cls = self.__class__
        result = cls.__new__(cls)
        memodict[id(self)] = result
        for k, v in self.__dict__.items():
            setattr(result, k, deepcopy(v, memodict))
        return result


    def __eq__(self, other):
        if len(self.movements) != len(other.movements):
            return False

        for i in range(len(self.movements)):
            if self.movements[i] != other.movements[i]:
                return False

        if self.current_node_id != other.current_node_id:
            return False

        if self.movement_idx != other.movement_idx:
            return False

        if self.current_movement != other.current_movement:
            return False

        for idx, nodes in self.nodes_per_movement.items():
            if nodes != other.nodes_per_movement[idx]:
                return False

        for idx, dist in self.dist_per_movement.items():
            if dist - other.dist_per_movement[idx] > 1e-1:
                return False

        for idx, dist in self.org_movement_dist.items():
            if dist - other.org_movement_dist[idx] > 1e-1:
                return False

        if self.expansion_mode != other.expansion_mode:
            return False

        return True

def simplify_segments(segments):
    """
    Convert the segments into list of movements.
    :param segments:
    :return:
    """
    movements = []

    prev_angle = None
    curr_dist = 0
    for seg in segments:
        current_angle = compute_latlong_angle(seg[0][0], seg[0][1], seg[1][0], seg[1][1])
        seg_dist = geodesic((seg[0][0], seg[0][1]), (seg[1][0], seg[1][1])).meters

        if prev_angle is None:
            prev_angle = current_angle
            curr_dist += seg_dist
            continue


        angle_diff = min(abs(prev_angle - current_angle), abs(360 - abs(prev_angle - current_angle)))
        if angle_diff > 45:
            # Close the current movement, start a new one.
            movements.append(Movement(angle=prev_angle , magnitude=curr_dist))
            curr_dist = seg_dist
            prev_angle = current_angle
        else:
            curr_dist += seg_dist

    if curr_dist != 0:
        movements.append(Movement(angle=prev_angle, magnitude=curr_dist))

    return movements


def find_matching_node_for_movement(movement, start_node, start_node_id, nodes_ways, nodes_id_to_location, threshold=50):
    min_angle_diff = float('inf')
    best_neighbor = None
    best_distance = None
    best_angle = None
    start_node = (float(start_node[0]), float(start_node[1]))
    for neighbor_id in nodes_ways[start_node_id]:
        neighbor_node = nodes_id_to_location[neighbor_id]
        neighbor_node = (float(neighbor_node[0]), float(neighbor_node[1]))
        nodes_angle = compute_latlong_angle(start_node[0], start_node[1], neighbor_node[0],
                                            neighbor_node[1])
        angle_diff = min(abs(360 - abs(nodes_angle - movement.angle)),abs(nodes_angle - movement.angle))
        if angle_diff < min_angle_diff:
            min_angle_diff = angle_diff
            best_neighbor = neighbor_id
            best_angle = nodes_angle
            best_distance = geodesic(start_node, (neighbor_node[0], neighbor_node[1])).meters

    logging.info("Most likely match, min angle diff: {0}".format(min_angle_diff))
    if min_angle_diff > threshold:
        return None, None, None

    return best_neighbor, best_distance, best_angle

def brute_algo(segments, current_location, nodes_manager, threshold_angle=40):
    """
    Given a starting location and segments, get a list of the possible routes.
    :param segments:
    :param current_location:
    :param nodes_manager:
    :param threshold_angle:
    :return:
    """
    nodes_ways = nodes_manager.get_nodes_ways()
    nodes_id_to_location = nodes_manager.get_nodes_map()

    movements = simplify_segments(segments)
    logging.info("Initial Movements: {0}".format(movements))
    if len(movements) == 0:
        raise Exception("No Movements in given segments")

    current_location = get_closest_node(current_location, nodes_manager.get_nodes())
    current_location = (float(current_location[0]), float(current_location[1]))

    possible_routes = []
    route_stack = []

    initial_route = PossibleRoute(movements, nodes_manager.get_node_id(current_location))
    route_stack.append(initial_route)

    # We don't want to iterate over the same route again and again.
    previous_seen_routes = []

    # Iterate until the stack is completely empty.
    while len(route_stack) != 0:
        logging.info("Possible Routes: {0}, Routes in Stack: {1}".format(len(possible_routes), len(route_stack)))
        curr_route = route_stack[0]
        previous_seen_routes.append(curr_route)

        # Check if we can continue in the current movement
        node_id = curr_route.current_node_id
        node_location = nodes_id_to_location[node_id]
        movement = curr_route.current_movement

        # Stop when the route is not valid.
        if not curr_route.is_valid():
            route_stack.pop(0)
            continue



        # Get best matching neighbor for current movement. We stop when we covered at least the distance of the movement
        # at least thats what we initially aspire to do.
        next_node, dist, angle = find_matching_node_for_movement(movement,node_location, node_id, nodes_ways, nodes_id_to_location)
        if next_node is not None:
            logging.info("Advancing current route in the direction of the movement.")

            if curr_route.movement_idx not in curr_route.nodes_per_movement:
                curr_route.current_movement = Movement(angle=angle, magnitude=movement.magnitude)


            curr_route.add_node_for_movement(next_node, dist)

            if curr_route.expansion_mode:
                # Create a copy of the route where we continue on the same expanding route.
                copied_route = deepcopy(curr_route)
                route_stack.append(copied_route)

                # Finish movement on the current route.
                curr_route.finished_movement()

            # If the coverage is done, add this as a possible route and pop from the stack.
            if curr_route.done:
                logging.info("Current route is done, removing from stack.")
                if curr_route not in possible_routes:
                    possible_routes.append(curr_route)
                route_stack.pop(0)

        else:
            """
            If there is no next node we immediately pop the current route from the stack and we have two
            main options of things we can do:
            1. Check if we can directly jump to the next movement if there is one. If there is none, just stop.
            2. Go back according to our route and check when we could jump to the next movement where we didn't choose
                to do so.
            """


            route_stack.pop(0)

            # If we are on the last movement, take this as a possible option.
            if curr_route.is_on_last_movement():
                logging.info("Route is on the last movement, finished.")
                if curr_route not in possible_routes:
                    possible_routes.append(curr_route)
            else:

                # Check if there is a way we can jump straight to the next movement. Done only if we moved a bit
                # in the current movement.

                if curr_route.num_nodes_of_movement(curr_route.movement_idx) > 0:
                    next_movement = curr_route.get_next_movement()
                    jump_node, jump_dist, jump_angle = find_matching_node_for_movement(next_movement, node_location, node_id, nodes_ways,
                                                                           nodes_id_to_location)

                    if jump_node is not None:
                        # If we can indeed cut, add this optional route to the stack.
                        copied_route = deepcopy(curr_route)
                        copied_route.advance_movement()
                        copied_route.current_movement = Movement(angle=jump_angle, magnitude=copied_route.current_movement.magnitude)
                        copied_route.add_node_for_movement(jump_node, jump_dist)


                        if copied_route not in previous_seen_routes:
                            logging.info("Adding option of cutting to the next movement, adding to the stack.")
                            if copied_route.done:
                                if copied_route not in possible_routes:
                                    possible_routes.append(copied_route)
                            else:
                                route_stack.append(copied_route)

                # Another option is to backtrack our path until the first instance where we could cut
                # to the next movement but we chose not to.
                copied_route = deepcopy(curr_route)
                chosen_node = None
                chosen_dist = None
                logging.info("Checking option of going backwards and cutting, current number of actions: {0}"
                             .format(copied_route.num_actions()))


                while chosen_node is None and copied_route.num_actions() != 0:
                    copied_route.backtrack()

                    next_movement = copied_route.get_next_movement()
                    curr_node_id = copied_route.current_node_id
                    chosen_node, chosen_dist, chosen_angle = find_matching_node_for_movement(next_movement,
                                                                               nodes_id_to_location[curr_node_id],
                                                                               curr_node_id,
                                                                               nodes_ways, nodes_id_to_location)

                    if chosen_node is not None:
                        copied_route.advance_movement()
                        copied_route.current_movement = Movement(angle=chosen_angle,
                                                                 magnitude=copied_route.current_movement.magnitude)
                        copied_route.add_node_for_movement(chosen_node, chosen_dist)
                        if copied_route not in previous_seen_routes:
                            logging.info("Found option to go backwards, number of actions: {0}".format(copied_route.num_actions()))
                            if copied_route.done:
                                if copied_route not in possible_routes:
                                    possible_routes.append(copied_route)
                            else:
                                route_stack.append(copied_route)

                                # TODO- Check the effect of removing the break and going over all the possible routes.
                            break
                        else:
                            copied_route.backtrack()
                            chosen_node = None

                # Extend the current movement, allow extension up to two times the original length of the movement.
                # Do this only when you can go back a movement.
                if curr_route.movement_idx > 0:
                    logging.info("Expanding movement {0}".format(curr_route.movement_idx))
                    copied_route = deepcopy(curr_route)
                    copied_route.set_previous_movement()
                    copied_route.extend_movement()
                    route_stack.append(copied_route)


    print("Number of possible routes: ", len(possible_routes))
    if len(possible_routes) == 0:
        print("NO ROUTES")
        return []

    # best_route = movement_based_arbitrator(possible_routes, movements, nodes_id_to_location)
    # return best_route.get_full_location_path(nodes_id_to_location)
    return possible_routes[2].get_full_location_path(nodes_id_to_location)

def convert_segs(segments):
    new_segs = []
    for seg in segments:
        new_segs.append(((float(seg[0][0]), float(seg[0][1])), (float(seg[1][0]), float(seg[1][1]))))
    return new_segs

if __name__=="__main__":
    new_segs = []
    segs = [((Decimal('32.06029175849473489279262139461934566497802734375'), Decimal('34.7696575441984094823055784218013286590576171875')), (Decimal('32.059725945107658162669395096600055694580078125'), Decimal('34.7696575441984094823055784218013286590576171875'))), ((Decimal('32.059725945107658162669395096600055694580078125'), Decimal('34.7696575441984094823055784218013286590576171875')), (Decimal('32.059301585067345286006457172334194183349609375'), Decimal('34.7696575441984094823055784218013286590576171875'))), ((Decimal('32.059301585067345286006457172334194183349609375'), Decimal('34.7696575441984094823055784218013286590576171875')), (Decimal('32.059018678373803368231165222823619842529296875'), Decimal('34.7696575441984094823055784218013286590576171875'))), ((Decimal('32.059018678373803368231165222823619842529296875'), Decimal('34.7696575441984094823055784218013286590576171875')), (Decimal('32.058735771680261450455873273313045501708984375'), Decimal('34.7696575441984094823055784218013286590576171875'))), ((Decimal('32.058735771680261450455873273313045501708984375'), Decimal('34.7696575441984094823055784218013286590576171875')), (Decimal('32.058452864986719532680581323802471160888671875'), Decimal('34.7696575441984094823055784218013286590576171875'))), ((Decimal('32.058452864986719532680581323802471160888671875'), Decimal('34.7696575441984094823055784218013286590576171875')), (Decimal('32.058169958293177614905289374291896820068359375'), Decimal('34.7696575441984094823055784218013286590576171875'))), ((Decimal('32.058169958293177614905289374291896820068359375'), Decimal('34.7696575441984094823055784218013286590576171875')), (Decimal('32.057887051599635697129997424781322479248046875'), Decimal('34.7696575441984094823055784218013286590576171875'))), ((Decimal('32.057887051599635697129997424781322479248046875'), Decimal('34.7696575441984094823055784218013286590576171875')), (Decimal('32.057604144906093779354705475270748138427734375'), Decimal('34.7696575441984094823055784218013286590576171875'))), ((Decimal('32.057604144906093779354705475270748138427734375'), Decimal('34.7696575441984094823055784218013286590576171875')), (Decimal('32.05706940475077004748527542687952518463134765625'), Decimal('34.76978349728972972343399305827915668487548828125'))), ((Decimal('32.05706940475077004748527542687952518463134765625'), Decimal('34.76978349728972972343399305827915668487548828125')), (Decimal('32.0569396696951827152588521130383014678955078125'), Decimal('34.7704674087994618503216770477592945098876953125'))), ((Decimal('32.0569396696951827152588521130383014678955078125'), Decimal('34.7704674087994618503216770477592945098876953125')), (Decimal('32.0569396696951827152588521130383014678955078125'), Decimal('34.7711350186315399923842051066458225250244140625'))), ((Decimal('32.0569396696951827152588521130383014678955078125'), Decimal('34.7711350186315399923842051066458225250244140625')), (Decimal('32.0569396696951827152588521130383014678955078125'), Decimal('34.771635726005598598931101150810718536376953125'))), ((Decimal('32.0569396696951827152588521130383014678955078125'), Decimal('34.771635726005598598931101150810718536376953125')), (Decimal('32.0569396696951827152588521130383014678955078125'), Decimal('34.77199863398156054472565301693975925445556640625')))]
    for seg in segs:
        new_segs.append(((float(seg[0][0]),float(seg[0][1])), (float(seg[1][0]),float(seg[1][1]))))

    simplify_segments(new_segs)
