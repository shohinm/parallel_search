import numpy as np
import matplotlib.pyplot as plt


n_x = 300
n_y = 300

m = np.zeros((n_x, n_y))

d = 10

rects = [[125, 175, 120-d, 125-d],
		[170, 175, 125-d, 175+d],
		[125, 175, 170+d, 175+d]]


for rect in rects:
	m[rect[0]:rect[1], rect[2]:rect[3]] = 1


np.savetxt("binary_map.txt", m, fmt='%d')
plt.imshow(np.transpose(m))
plt.show()







