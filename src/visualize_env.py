import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import cv2


# define geometry
park_width = 2.5
park_length = 5.5

ratio = 10
offset = 6

# for test reason
car_position = [100, 50]



def read_map_data(map_file):
    map_data = np.loadtxt(map_file, delimiter=',', skiprows=1)
    x = map_data[:, 0]
    y = map_data[:, 1]
    theta = map_data[:, 2]
    occupied = map_data[:, 3]
    return x,y,theta,occupied


def getRectanglePointsFromCenter(center_x, center_y):
    pt1_x = int((center_x - park_length / 2) * ratio)
    pt1_y = int((center_y - park_width / 2) * ratio)

    pt2_x = int((center_x + park_length / 2) * ratio)
    pt2_y = int((center_y + park_width / 2) * ratio)
    return pt1_x,pt1_y, pt2_x,pt2_y

def draw_env(img):

    map_file = "map.csv"
    x, y, theta, occupied = read_map_data(map_file)
    n = len(x)

    rectangcles = np.zeros((n, 4))
    for i in range(n):
        rectangcles[i] = getRectanglePointsFromCenter(x[i], y[i])

    plt.figure(figsize=(32, 20))

    # draw rectangles
    for i in range(n):

        pt1_x, pt1_y, pt2_x, pt2_y = rectangcles[i]
        cv2.rectangle(img, (int(pt1_x), int(pt1_y)), (int(pt2_x), int(pt2_y)),
                      (255, 250, 250), 1)
        if occupied[i] == 1:  # occupied
            cv2.rectangle(img,
                          (int(pt1_x) + offset, int(pt1_y) + offset),
                          (int(pt2_x) - offset, int(pt2_y) - offset),
                          (173, 168, 137), cv2.FILLED)

        return img

def main():

    img = np.ones((280, 450, 3), np.uint8) * 80
    img = draw_env(img)

    plt.imshow(img, 'brg')
    plt.show()

    return


if __name__ == '__main__':
    main()








