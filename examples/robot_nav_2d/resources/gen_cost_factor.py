import numpy as np
import os
import pdb

dim = {'hrt201n': [305, 294], 'den501d': [338, 320], 'den520d': [257, 256],  'ht_chantry':  [141, 162], 'brc203d': [391, 274]}

scale = 5

for map_name, dims in dim.items():
	print("Map: {} | dims: {}".format(map_name, np.multiply(dims,scale)))
	cost_mat = np.random.uniform(low=1, high=100, size=np.multiply(dims,scale))
	np.savetxt(os.path.join(map_name, map_name +'_cost_factor.map'), cost_mat, fmt='%d')
