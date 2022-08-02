import numpy as np
import matplotlib.pyplot as plt


def make_map(d1, d2, fname):

    n_x = 300
    n_y = 300

    m = np.zeros((n_x, n_y))

    rects = [[125-d1, 175+d1, 120-d2, 125-d2],
            [170+d1, 175+d1, 125-d2, 175+d2],
            [125-d1, 175+d1, 170+d2, 175+d2]]


    for rect in rects:
        m[rect[0]:rect[1], rect[2]:rect[3]] = 1


    np.savetxt(fname, m, fmt='%d')
    # plt.imshow(np.transpose(m))
    # plt.show()




if __name__ == "__main__":
    make_map(0, 0, 'binary_map_small.txt');
    make_map(10, 20, 'binary_map_medium.txt');
    make_map(20, 40, 'binary_map_large.txt');








