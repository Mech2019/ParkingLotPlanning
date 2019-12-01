import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import cv2

# define geometry
park_width = 2.5
park_length = 5.5

ratio = 10
offset = 6

map_file = "map.csv"
map_data = np.loadtxt(map_file, delimiter=',', skiprows=1)

print(map_data.shape)

x = map_data[:, 0]
y = map_data[:, 1]
theta = map_data[:, 2]
occupied = map_data[:, 3]

def getRectanglePointsFromCenter(center_x, center_y):
    pt1_x = int((center_x - park_length / 2) * ratio)
    pt1_y = int((center_y - park_width / 2) * ratio)

    pt2_x = int((center_x + park_length / 2) * ratio)
    pt2_y = int((center_y + park_width / 2) * ratio)

    return pt1_x,pt1_y, pt2_x,pt2_y

rectangcles = np.zeros()


plt.figure(figsize=(32, 20))

img = np.ones((280,450,3),np.uint8) * 80

# draw rectangles
for i in range(len(x)):
    pt1_x, pt1_y, pt2_x, pt2_y = getRectanglePointsFromCenter(x[i], y[i])
    if occupied[i] == 0: # empty
        cv2.rectangle(img,(pt1_x, pt1_y),(pt2_x, pt2_y),(255, 250, 250),1)
    else:
        cv2.rectangle(img, (pt1_x, pt1_y), (pt2_x, pt2_y), (255, 250, 250),
                      1)
        cv2.rectangle(img,
                      (pt1_x + offset, pt1_y + offset),
                      (pt2_x - offset, pt2_y - offset),
                      (55, 255, 155), cv2.FILLED)

plt.imshow(img,'brg')

plt.show()






