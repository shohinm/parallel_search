import numpy as np
import os
import pdb

dim = {'hrt201n': [305, 294], 'den501d': [338, 320], 'den520d': [257, 256],  'ht_chantry':  [141, 162], 'brc203d': [391, 274]}

scale_vec = [5, 5, 5, 10, 5]

map_idx = 0
for map_name, dims in dim.items():
	print("Map: {} | dims: {}".format(map_name, np.multiply(dims, scale_vec[map_idx])))
	cost_mat = np.random.uniform(low=1, high=100, size=np.multiply(dims, scale_vec[map_idx]))
	np.savetxt(os.path.join(map_name, map_name +'_cost_factor.map'), cost_mat, fmt='%d')
