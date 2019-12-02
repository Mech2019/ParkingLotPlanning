import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import cv2
import math
import matplotlib.animation as animation


# define geometry
park_width = 2.5
park_length = 5.5
car_width = 2
car_length = 4

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


def draw_rectangle(img, centre, theta, width, height):
    # theta = np.radians(theta)
    c, s = np.cos(theta), np.sin(theta)
    R = np.matrix('{} {}; {} {}'.format(c, -s, s, c))
    # print(R)
    p1 = [+ width / 2, + height / 2]
    p2 = [- width / 2, + height / 2]
    p3 = [- width / 2, - height / 2]
    p4 = [+ width / 2, - height / 2]
    p1_new = np.dot(p1, R) + centre
    p2_new = np.dot(p2, R) + centre
    p3_new = np.dot(p3, R) + centre
    p4_new = np.dot(p4, R) + centre

    img = cv2.line(img, (int(p1_new[0, 0]), int(p1_new[0, 1])),
                   (int(p2_new[0, 0]), int(p2_new[0, 1])), (255, 0, 0), 1)
    img = cv2.line(img, (int(p2_new[0, 0]), int(p2_new[0, 1])),
                   (int(p3_new[0, 0]), int(p3_new[0, 1])), (255, 0, 0), 1)
    img = cv2.line(img, (int(p3_new[0, 0]), int(p3_new[0, 1])),
                   (int(p4_new[0, 0]), int(p4_new[0, 1])), (255, 0, 0), 1)
    img = cv2.line(img, (int(p4_new[0, 0]), int(p4_new[0, 1])),
                   (int(p1_new[0, 0]), int(p1_new[0, 1])), (255, 0, 0), 1)
    return img

def drawCar(img, center_x, center_y, theta):
    cv2.circle(img, (int(center_x), int(center_y)), 5, (50, 168, 54), -1)
    img = draw_rectangle(img, [center_x, center_y], theta, car_length*ratio,
                         car_width*ratio)
    return img


def main():

    x, y, theta, occupied = read_map_data(map_file)
    n = len(x)

    rectangles = np.zeros((n, 4))
    for i in range(n):
        rectangles[i] = getRectanglePointsFromCenter(x[i], y[i])

    plt.figure(figsize=(32, 20))
    img = np.ones((280, 450, 3), np.uint8) * 80

    # draw rectangles
    for i in range(n):
        pt1_x, pt1_y, pt2_x, pt2_y = rectangles[i]
        cv2.rectangle(img, (int(pt1_x), int(pt1_y)), (int(pt2_x), int(pt2_y)),
                      (255, 250, 250), 1)
        if occupied[i] == 1:  # occupied
            cv2.rectangle(img,
                          (int(pt1_x) + offset, int(pt1_y) + offset),
                          (int(pt2_x) - offset, int(pt2_y) - offset),
                          (173, 168, 137), cv2.FILLED)

    # draw car in current position
    # car_pos = np.zeros((n, 4))
    # car_pos = getCarPosFromCenter(car_traj_input[0],car_traj_input[1],
    #                               car_traj_input[2])
    #
    # pt1_x, pt1_y, pt2_x, pt2_y = car_pos
    # print("current car position: ", pt1_x, pt1_y, pt2_x, pt2_y)
    # cv2.rectangle(img, (int(pt1_x), int(pt1_y)), (int(pt2_x), int(pt2_y)),
    #               (255, 0, 0), cv2.FILLED)


    traj_num = car_traj_input.shape[0]

    for i in range(traj_num):
        print(i)
        car_pos = car_traj_input[i, :]
        center_x = car_pos[0] * 10
        center_y = car_pos[1] * 10
        theta = car_pos[2]
        img = drawCar(img, center_x, center_y, theta)

    # ani = animation.FuncAnimation(fig, update_points, np.arange(0, traj_num),
    #                               interval=100, blit=True)
    plt.imshow(img, 'brg')
    plt.savefig("env.png")
    plt.show()
    return


if __name__ == '__main__':
    main()








