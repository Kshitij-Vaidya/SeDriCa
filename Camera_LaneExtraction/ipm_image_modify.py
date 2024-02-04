import cv2
import numpy as np

path = '/Users/kshitijvaidya/Desktop/VirtualEnvironment/Camera_LaneExtraction/IPM_Images/IPM_Image0.png'
image = cv2.imread(path)

leftSlope = (779 - 159) / (362 - 0)
rightSlope = (779 - 0) / (784 - 1039)

print(leftSlope)
print(rightSlope)


def imageModify(grey_image):
    grey_image = cv2.resize(grey_image, (1280, 720), interpolation = cv2.INTER_AREA)
    leftSlope = (719 - 158) / (327 - 0)
    rightSlope = (719 - 0) / (806 - 1038)

    # [0, 1038] [719, 806]
    x_upper = [1038, 1037, 1036, 1035, 1034, 1033, 1032]
    x_lower = [806, 805, 804, 803, 802, 801, 800]
    for x_u, x_l in zip(x_upper, x_lower):
        x_list = [i for i in range(x_l, x_u + 1)][-1::-1]
        y_list = [int(rightSlope * (i - x_u)) for i in x_list]
        for i in range(len(y_list)):
            if y_list[i] > 719:
                y_list[i] = 719

        for y, x in zip(y_list, x_list):
            grey_image[y, x] = 0

    # Moving from points : (158, 0) to (158, 6)
    # End points : (719, 327) to (719, 333)
    y_range = [327, 328, 329, 330, 331, 332, 333]
    x_range = [0, 1, 2, 3, 4, 5, 6]

    for y_pixel, x_pixel in zip(y_range, x_range):
        x_list = [i for i in range(0, y_pixel+1)]
        y_list = [int(((leftSlope * (i - x_pixel)) + y_pixel)) for i in x_list]
        for i in range(len(y_list)):
            if y_list[i] > 719:
                y_list[i] = 719

        for y, x in zip(y_list, x_list):
            grey_image[y, x] = 0

    return grey_image
   