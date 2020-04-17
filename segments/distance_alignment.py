import math

def calculate_distance(polyline):
    """
    Given a complete polyline of points in cartesian coordinates, compute the total distance accumulated by traveling
    based on the given polyline.
    :param geo_polyline:
    :return:
    """
    if len(polyline) <= 1:
        raise Exception("The input polyline should be of length of more than 1.")

    prev_point = polyline[0]
    total_dist = 0.0
    for idx in range(1, len(polyline)):
        total_dist += math.sqrt((polyline[idx][0] - prev_point[0]) ** 2 + (polyline[idx][1] - prev_point[1]) ** 2)
        prev_point = polyline[idx]

    return total_dist

def compute_target_from_new_source(source, old_target, new_source, dist):
    angle = math.atan2(old_target[1] - source[1], old_target[0] - source[0])
    if source[0] <= old_target[0] and source[1] <= old_target[1]:
        point =  (new_source[0] + dist * math.cos(angle), new_source[1] + dist * math.sin(angle))
    elif source[0] <= old_target[0] and source[1] > old_target[1]:
        point =  (new_source[0] + dist * math.cos(abs(angle)), new_source[1] - dist * math.sin(abs(angle)))
    elif source[0] > old_target[0] and source[1] <= old_target[1]:
        point =  (new_source[0] - dist * math.cos(math.pi - angle), new_source[1] + dist * math.sin(math.pi - angle))
    else:
        point =  (new_source[0] - dist * math.cos(angle - math.pi), new_source[1] - dist * math.sin(angle - math.pi))

    return point



def get_angle_from_point(source, target):
    angle = math.atan2(target[1] - source[1], target[0] - source[1])
    if source[1] <= target[1]:
        return math.pi - angle
    else:
        return angle + math.pi

def scale_route_to_distance(target_distance, polyline, average_distance=None):
    '''
    Perform scaling of the trajectory based on the target distance.
    Works only on a single polyline!
    :param target_distance:
    :param polyline:
    :return:
    '''
    if len(polyline) <= 1:
        raise Exception("The input polyline should be of length of more than 1.")

    initial_distance = calculate_distance(polyline)
    adjusted_polyline = [polyline[0]]
    prev_point = polyline[0]

    adjusted_idx = 1
    for idx in range(1, len(polyline)):
        dist = math.sqrt((polyline[idx][0] - prev_point[0]) ** 2 + (polyline[idx][1] - prev_point[1]) ** 2)
        ratio = dist / initial_distance
        target_dist = ratio * target_distance

        num_points = 0
        if average_distance is not None:
            num_points = target_dist / average_distance

        if int(math.floor(num_points)) == 0:
            new_point = compute_target_from_new_source(prev_point, polyline[idx], adjusted_polyline[adjusted_idx - 1], target_dist)
            adjusted_idx += 1
            adjusted_polyline.append(new_point)
        else:
            for i in range(int(math.floor(num_points))):
                new_point = compute_target_from_new_source(prev_point, polyline[idx], adjusted_polyline[adjusted_idx - 1], average_distance)
                adjusted_polyline.append(new_point)
                adjusted_idx += 1

            remaining_dist = target_dist - int(math.floor(num_points)) * average_distance
            if remaining_dist > 1:
                adjusted_polyline.append(compute_target_from_new_source(prev_point, polyline[idx], adjusted_polyline[adjusted_idx - 1], remaining_dist))
                adjusted_idx += 1

        prev_point = polyline[idx]

    return adjusted_polyline

if __name__=="__main__":
    first = (0,0)
    second = (2,-2)
    ang = math.atan2(second[1] - first[1], second[0] - first[0]) * 180 / math.pi
    print("Angle: ", ang)


