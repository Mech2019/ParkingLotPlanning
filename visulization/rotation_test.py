import matplotlib.pyplot as plt
import numpy as np
import cv2

def draw_rectangle(img, centre, theta, width, height):
    theta = np.radians(theta)
    c, s = np.cos(theta), np.sin(theta)
    R = np.matrix('{} {}; {} {}'.format(c, -s, s, c))
    # print(R)
    print(centre[0])
    p1 = [+ width / 2, + height / 2]
    p2 = [- width / 2, + height / 2]
    p3 = [- width / 2, - height / 2]
    p4 = [+ width / 2, - height / 2]
    p1_new = np.dot(p1, R) + centre
    p2_new = np.dot(p2, R) + centre
    p3_new = np.dot(p3, R) + centre
    p4_new = np.dot(p4, R) + centre
    print(p1_new)

    img = cv2.line(img, (int(p1_new[0, 0]), int(p1_new[0, 1])),
                   (int(p2_new[0, 0]), int(p2_new[0, 1])), (255, 0, 0), 1)
    img = cv2.line(img, (int(p2_new[0, 0]), int(p2_new[0, 1])),
                   (int(p3_new[0, 0]), int(p3_new[0, 1])), (255, 0, 0), 1)
    img = cv2.line(img, (int(p3_new[0, 0]), int(p3_new[0, 1])),
                   (int(p4_new[0, 0]), int(p4_new[0, 1])), (255, 0, 0), 1)
    img = cv2.line(img, (int(p4_new[0, 0]), int(p4_new[0, 1])),
                   (int(p1_new[0, 0]), int(p1_new[0, 1])), (255, 0, 0), 1)
    return img

image = plt.figure()

img = np.ones((280, 450, 3), np.uint8) * 80
theta = 1.55
centre = [100, 100]
width = 40
height = 20
img = draw_rectangle(img, centre, theta, width, height)

plt.imshow(img, 'brg')
plt.show()