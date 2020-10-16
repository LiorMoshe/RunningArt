
from scipy import ndimage
import numpy as np

"""
Given an image of the area and a list of realistic starting positions, run a convolution with the image of
the segments over the given area.
"""

def find_starting_location(area_image, segment_image, possible_starting_locations, segment_start):
    optimal_location = None
    highest_conv_score = float('-inf')


    result = np.zeros_like(area_image)
    ndimage.convolve(area_image, segment_image, output=result)

    for start in possible_starting_locations:
        curr_score = result[start[0], start[1]]
        print("Index: {0}, Score: {1}".format(start, curr_score))
        if curr_score > highest_conv_score:

            highest_conv_score = curr_score
            optimal_location = start

    print("Highest score ",highest_conv_score)
    print("Optimal Loc: {0}".format(optimal_location))
    return optimal_location

    # for start_loc in possible_starting_locations:
    #     # Check if this location is even viable given the area (based on dimensions)
    #     is_viable = False
    #
    #
    #     # If it is viable convolve.
    #     if is_viable:
    #         score = ndimage.convolve()



if __name__=="__main__":
    import numpy as np
    from scipy import ndimage


    x = np.random.random((2048, 2048)).astype(np.float32)
    print(x.shape)
    y = np.random.random((32, 32)).astype(np.float32)
    ndimage.convolve(x, y, output=x)
    print(x.shape)

    #
    # t = timeit.timeit(stmt='ndimage.convolve(x, y, output=x)', number=1,
    #                   setup="""
    #     import numpy as np
    #     from scipy import ndimage
    #     x = np.random.random((2048, 2048)).astype(np.float32)
    #     y = np.random.random((32, 32)).astype(np.float32)
    # """)
    # print(t)
