import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import cv2


# define geometry
park_width = 2.5
park_length = 5.5
car_width = 2
car_length = 4.5

ratio = 10
offset = 6

map_file = "map.csv"

# for test reason
# car_traj_input = [10, 26, 0]
plan_file = "plan_sample.txt"
car_traj_input = np.loadtxt(plan_file, delimiter=',', skiprows=0)

print(car_traj_input)


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

def getCarPosFromCenter(center_x, center_y, theta):
    pt1_x = int((center_x - car_length / 2) * ratio)
    pt1_y = int((center_y - car_width / 2) * ratio)

    pt2_x = int((center_x + car_length / 2) * ratio)
    pt2_y = int((center_y + car_width / 2) * ratio)
    return pt1_x,pt1_y, pt2_x,pt2_y



def main():

    x, y, theta, occupied = read_map_data(map_file)
    n = len(x)

    rectangcles = np.zeros((n, 4))
    for i in range(n):
        rectangcles[i] = getRectanglePointsFromCenter(x[i], y[i])

    plt.figure(figsize=(32, 20))
    img = np.ones((280, 450, 3), np.uint8) * 80

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

    # draw car in current position
    car_pos = np.zeros((n, 4))
    car_pos = getCarPosFromCenter(car_traj_input[0],car_traj_input[1],
                                  car_traj_input[2])

    pt1_x, pt1_y, pt2_x, pt2_y = car_pos
    print("current car position: ", pt1_x, pt1_y, pt2_x, pt2_y)
    cv2.rectangle(img, (int(pt1_x), int(pt1_y)), (int(pt2_x), int(pt2_y)),
                  (255, 0, 0), cv2.FILLED)


    plt.imshow(img, 'brg')
    plt.savefig("env.png")
    plt.show()
    return


if __name__ == '__main__':
    main()








