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

# image = plt.figure()
#
# img = np.ones((280, 450, 3), np.uint8) * 80
# theta = 1.55
# centre = [100, 100]
# width = 40
# height = 20
# img = draw_rectangle(img, centre, theta, width, height)
#
# plt.imshow(img, 'brg')
# plt.show()

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# %matplotlib inline

# x = np.linspace(0, 2*np.pi, 100)
# y = np.sin(x)
#
# fig = plt.figure(tight_layout=True)
# plt.plot(x,y)
# plt.grid(ls="--")
# plt.show()


def update_points(num):
    point_ani.set_data(x[num], y[num])
    return point_ani,


# x = np.linspace(0, 2*np.pi, 100)
# y = np.sin(x)
#
# fig = plt.figure(tight_layout=True)
# plt.plot(x,y)
# point_ani, = plt.plot(x[0], y[0], "ro")
# plt.grid(ls="--")
#
# ani = animation.FuncAnimation(fig, update_points, np.arange(0, 100), interval=100, blit=True)
#
# # ani.save('sin_test2.gif', writer='imagemagick', fps=10)
# plt.show()


import numpy as np
from cv2 import VideoWriter, VideoWriter_fourcc

height = 720
width = 1280
FPS = 24
seconds = 10

fourcc = VideoWriter_fourcc(*'MP42')
video = VideoWriter('./noise.avi', fourcc, float(FPS), (width, height))

for _ in range(FPS*seconds):
    frame = np.random.randint(0, 256,
                              (height, width, 3),
                              dtype=np.uint8)
    video.write(frame)
video.release()