import cv2
import numpy as np
import math
from PIL import Image
from skimage import img_as_float, color, morphology


def _palette_is_grayscale(pil_image):
    """Return True if PIL image in palette mode is grayscale.
    Parameters
    ----------
    pil_image : PIL image
        PIL Image that is in Palette mode.
    Returns
    -------
    is_grayscale : bool
        True if all colors in image palette are gray.
    """
    if pil_image.mode != 'P':
        raise ValueError('pil_image.mode must be equal to "P".')
    # get palette as an array with R, G, B columns
    palette = np.asarray(pil_image.getpalette()).reshape((256, 3))
    # Not all palette colors are used; unused colors have junk values.
    start, stop = pil_image.getextrema()
    valid_palette = palette[start:stop + 1]
    # Image is grayscale if channel differences (R - G and G - B)
    # are all zero.
    return np.allclose(np.diff(valid_palette), 0)

# Converts PIL image to skimage.
def pil_to_ndarray(image, dtype=None, img_num=None):
    """Import a PIL Image object to an ndarray, in memory.
    Parameters
    ----------
    Refer to ``imread``.
    """
    try:
        # this will raise an IOError if the file is not readable
        image.getdata()[0]
    except IOError as e:
        site = "http://pillow.readthedocs.org/en/latest/installation.html#external-libraries"
        pillow_error_message = str(e)
        error_message = ('Could not load "%s" \n'
                         'Reason: "%s"\n'
                         'Please see documentation at: %s'
                         % (image.filename, pillow_error_message, site))
        raise ValueError(error_message)
    frames = []
    grayscale = None
    i = 0
    while 1:
        try:
            image.seek(i)
        except EOFError:
            break

        frame = image

        if img_num is not None and img_num != i:
            image.getdata()[0]
            i += 1
            continue

        if image.format == 'PNG' and image.mode == 'I' and dtype is None:
            dtype = 'uint16'

        if image.mode == 'P':
            if grayscale is None:
                grayscale = _palette_is_grayscale(image)

            if grayscale:
                frame = image.convert('L')
            else:
                if image.format == 'PNG' and 'transparency' in image.info:
                    frame = image.convert('RGBA')
                else:
                    frame = image.convert('RGB')

        elif image.mode == '1':
            frame = image.convert('L')

        elif 'A' in image.mode:
            frame = image.convert('RGBA')

        elif image.mode == 'CMYK':
            frame = image.convert('RGB')

        if image.mode.startswith('I;16'):
            shape = image.size
            dtype = '>u2' if image.mode.endswith('B') else '<u2'
            if 'S' in image.mode:
                dtype = dtype.replace('u', 'i')
            frame = np.fromstring(frame.tobytes(), dtype)
            frame.shape = shape[::-1]

        else:
            frame = np.array(frame, dtype=dtype)

        frames.append(frame)
        i += 1

        if img_num is not None:
            break

    if hasattr(image, 'fp') and image.fp:
        image.fp.close()

    if img_num is None and len(frames) > 1:
        return np.array(frames)
    elif frames:
        return frames[0]
    elif img_num:
        raise IndexError('Could not find image  #%s' % img_num)

def findContours(img):
    """
    Receives a PIL image and returns its contour points.
    :param img:
    :return:
    """
    img = img.convert('RGB')
    img = np.array(img)
    # Convert RGB to BGR
    img = img[:, :, ::-1].copy()

    # Grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    contours, hierarchy = cv2.findContours(gray,
                                           cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    # Convert the contours to a matching format.

    lines = []
    for i in range(len(contours)):
        polyline = []
        contours[i] = contours[i].reshape((-1, 2))
        for j in range(contours[i].shape[0]):
            polyline.append((contours[i][j][0], contours[i][j][1]))
        lines.append(polyline)

    return lines

def find_thick_contours(img):
    image = img_as_float(color.rgb2gray(pil_to_ndarray(img)))
    image_binary = image < 0.5
    IM = img_frombytes(morphology.thin(image_binary))
    return findContours(IM)

def img_frombytes(data):
    size = data.shape[::-1]
    databytes = np.packbits(data, axis=1)
    return Image.frombytes(mode='1', size=size, data=databytes)


def visualize(lines):
    import turtle
    wn = turtle.Screen()
    # wn.setup(200, 200)
    t = turtle.Turtle()
    t.speed(1)
    t.pencolor('red')
    t.pd()
    for i in range(0,len(lines)):
        for p in lines[i]:
            t.goto(p[0],p[1])
            t.pencolor('black')
        t.pencolor('red')
    turtle.mainloop()


def segments_averaging(segments):
    new_segments = []
    centers = []
    idx_to_center = {}
    center_to_idx = {}
    center_to_participants = {}
    point_to_idx = {}
    changed = False


    curr_idx = 0
    for polyline in segments:
        seen = []
        for point in polyline:
            if point not in seen:
                seen.append(point)
                closest = None
                closest_dist = float('inf')

                for center in centers:
                    dist = math.sqrt((center[0] - point[0]) ** 2 + (center[1] - point[1]) ** 2)
                    if dist < closest_dist:
                        closest = center
                        closest_dist = dist

                if closest is None or closest_dist > 10:
                    centers.append(point)
                    center_to_participants[point] = 1

                    idx_to_center[curr_idx] = point
                    center_to_idx[point] = curr_idx
                    point_to_idx[point] = curr_idx

                    # Increment the idx.
                    curr_idx += 1
                elif closest_dist > 1:
                    # There is some change to the centers, notify.
                    changed = True

                    idx = center_to_idx[closest]
                    del idx_to_center[idx]
                    centers.remove(closest)
                    n = center_to_participants[closest]
                    new_center = ((n / (n+1)) * closest[0] + (1 / (n+1)) * point[0],
                                  (n / (n+1)) * closest[1] + (1 / (n+1)) * point[1])
                    del center_to_participants[closest]
                    center_to_participants[new_center] = n+1
                    centers.append(new_center)
                    idx_to_center[idx] = new_center
                    point_to_idx[point] = idx
                    center_to_idx[new_center] = idx
                else:
                    point_to_idx[point] = center_to_idx[closest]

        new_polyline = []
        print("Old Polyline: ", polyline)
        for point in polyline:
            prev_point = None
            if len(new_polyline) > 0:
                prev_point = new_polyline[len(new_polyline) - 1]

            current_center = idx_to_center[point_to_idx[point]]
            if current_center != prev_point:
                new_polyline.append(idx_to_center[point_to_idx[point]])
        print("New Polyline: ", new_polyline)
        new_segments.append(new_polyline)


    return new_segments, changed

# def remove_redundant_segments():


# function getDistanceFromLatLonInKm(lat1,lon1,lat2,lon2) {
#   var R = 6371; // Radius of the earth in km
#   var dLat = deg2rad(lat2-lat1);  // deg2rad below
#   var dLon = deg2rad(lon2-lon1);
#   var a =
#     Math.sin(dLat/2) * Math.sin(dLat/2) +
#     Math.cos(deg2rad(lat1)) * Math.cos(deg2rad(lat2)) *
#     Math.sin(dLon/2) * Math.sin(dLon/2)
#     ;
#   var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
#   var d = R * c; // Distance in km
#   return d;
# }
#
# function deg2rad(deg) {
#   return deg * (Math.PI/180)
# }

def get_lat_long_dist(lat1, lon1, lat2, lon2):
    deg2rad = lambda deg: deg * math.pi / 180
    R = 6371137.0
    dLat = deg2rad(lat2 - lat1)
    dLon = deg2rad(lon2 - lon1)
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(deg2rad(lat1)) * math.cos(deg2rad(lat2)) * \
        math.sin(dLon / 2) * math.sin(dLon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = R * c
    return d

def connect_letters(starting_location, segments):
    '''
    Given the segments and the starting locations, connect them together based on their closest points.
    :param starting_location:
    :param segments:
    :return:
    '''
    new_segments = []
    idx_to_dist = {}
    for idx, segment in enumerate(segments):
        dist = get_lat_long_dist(starting_location[0], starting_location[1], segment[0][0], segment[0][1])
        idx_to_dist[idx] = dist

    sorted_idx = {k: v for k, v in sorted(idx_to_dist.items(), key=lambda item: item[1])}
    sorted_indices = list(sorted_idx.keys())
    print("Sorted: ", sorted_indices)
    for i, idx in enumerate(sorted_indices):
        new_segments.append(segments[idx])
        if i < len(sorted_indices) - 1:
            current_polyline = segments[idx]
            next_polyline = segments[sorted_indices[i + 1]]

            smallest_dist = float('inf')
            new_polyline = []
            for p1 in current_polyline:
                for p2 in next_polyline:
                    dist = get_lat_long_dist(p1[0], p1[1], p2[0], p2[1])
                    if dist < smallest_dist:
                        smallest_dist = dist
                        new_polyline = [p1, p2]

            new_segments.append(new_polyline)


    return new_segments

if __name__=="__main__":

    # Let's load a simple image with 3 black squares

    image = Image.open('pil_text_font_l.png')

    lines = find_thick_contours(image)
    while True:
        lines, changed = segments_averaging(lines)
        if not changed:
            break

    # print("Lines: ", lines)
    for line in lines:
        print("Line: ", line)


    # Run turtle visualization
    visualize(lines)





