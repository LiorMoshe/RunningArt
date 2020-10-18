from location.image_generator import *
from location.spread_matrix import *

def save_image(image_name, image_data):
    img = Image.new('1', (image_data.shape[0], image_data.shape[1]))
    pixels = img.load()
    for i in range(img.size[0]):
        for j in range(img.size[1]):
            tmp = int(image_data[i, j].item())
            pixels[i, j] = tmp

    # TODO- Given image is rotated by 90 degrees.
    img = img.transpose(Image.ROTATE_90)
    img.save(image_name)

def get_optimal_start(initial_pos, segments, distance, nodes_manager, nearest_one_mode=True, image_scale=1):
    """
    Given an initial pos, the required segments and the distance of our run. Return the optimal starting position.
    :param segments:
    :param initial_pos:
    :param distance: Distance in km
    :return:
    """
    coordinates = bbox_calculation(initial_pos, distance)
    dimension = int(distance * 1000 * 2)
    print("Coordinates: ", coordinates)
    print("Input Segments: ", segments)
    segments_image = generate_segments_image(segments, coordinates, dimension, image_scale)
    binary_data, indices = generate_binary_matrix(coordinates, distance, nodes_manager, scale=image_scale)

    print("Saving Map Image")
    save_image("map_img.png", binary_data)
    print("Saving Segments Image")
    save_image("seg_img.png", segments_image)

    if nearest_one_mode:
        print("Operating NearestOne Mode.")
        binary_data = nearestOne(binary_data)

    print("Finding optimal starting loc.")
    opt = find_starting_location(binary_data, segments_image, indices, (0, 0))
    return get_position_from_indices((coordinates[0], coordinates[1]), opt, image_scale)
