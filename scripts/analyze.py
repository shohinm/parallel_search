import numpy as np
import pdb
from collections import defaultdict
# import matplotlib.pyplot as plt
import argparse
import os

def relevant_indices(alg, data, expansion_thresh, time_thresh):
    idx = np.array(np.where((data['cost'] > 0) & (data['state_expansions'] > expansion_thresh) & (data['time'] <= time_thresh))).squeeze()
        
    success = np.count_nonzero((data["cost"] > 0) & (data['time'] <= time_thresh))/float(data["cost"].shape[0])
    success_idx = np.array(np.where(data["cost"] != -1))

    preprocess_success = success

    if 'preprocess_cost' in data.keys():
        preprocess_success = np.count_nonzero((data["preprocess_cost"] > 0) & (data['time'] <= time_thresh))/float(data["preprocess_cost"].shape[0])

    # pdb.set_trace()
    return idx, success, success_idx, preprocess_success


def prune_failures(data):
    # pdb.set_trace()
    idx = np.array(np.where(data["cost"] != -1)).squeeze()
    success = np.count_nonzero(data["cost"] != -1)/float(data["cost"].shape[0])
    success_idx = np.array(np.where(data["cost"] != -1))
    return idx, success, success_idx


def sanity_check(data, expansion_thresh):
    result = True;
    for alg in data.keys():
        # result = result and (np.count_nonzero(data[alg]['cost'] < expansion_thresh) == 0) 
        result = result and (np.count_nonzero(data[alg]['cost'] == -1) == 0)

    result = result and np.array_equal(data['wastar']['id'], data['pase']['id']) and np.array_equal(data['wastar']['id'], data['gepase']['id'])
    # pdb.set_trace()
    return result


def print_stats(insat, pinsat, rrt, epase, expansion_thresh, time_thresh):
    print("\n\n--------------- Cost thresh: {} ---------------".format(expansion_thresh))

    d = {}
    d['insat'] = {'id' : insat[:, 0], 'time' : insat[:, 1], 'cost' : insat[:, 2], 
    'length' : insat[:, 3], 'state_expansions' : insat[:, 4], 'edge_expansions' : insat[:, 5]} 
    d['pinsat'] = {'id' : pinsat[:, 0], 'time' : pinsat[:, 1], 'cost' : pinsat[:, 2],
    'length' : pinsat[:, 3], 'state_expansions' : pinsat[:, 4], 'edge_expansions' : pinsat[:, 5]} 
    d['rrt'] = {'id' : rrt[:, 0], 'time' : rrt[:, 1], 'cost' : rrt[:, 2], 'length' : rrt[:, 3],
    'state_expansions' : rrt[:, 4], 'edge_expansions' : rrt[:, 5], 'preprocess_cost' : rrt[:, 8]} 
    d['epase'] = {'id' : epase[:, 0], 'time' : epase[:, 1], 'cost' : epase[:, 2], 'length' : epase[:, 3],
    'state_expansions' : epase[:, 4], 'edge_expansions' : epase[:, 5], 'preprocess_cost' : epase[:, 8]} 

    insat_idx, insat_success, insat_success_idx, insat_preprocess_success = relevant_indices('insat', d['insat'], expansion_thresh, time_thresh)
    pinsat_idx, pinsat_success, pinsat_success_idx, pinsat_preprocess_success = relevant_indices('pinsat', d['pinsat'], expansion_thresh, time_thresh)
    rrt_idx, rrt_success, rrt_success_idx, rrt_preprocess_success = relevant_indices('rrt', d['rrt'], -1, time_thresh)
    epase_idx, epase_success, epase_success_idx, epase_preprocess_success = relevant_indices('epase', d['epase'], -1, time_thresh)

    insat = insat[insat_idx, :]
    pinsat = pinsat[pinsat_idx, :]
    rrt = rrt[rrt_idx, :]
    epase = epase[epase_idx, :]

    # rrt_idx, rrt_success, rrt_success_idx  = prune_failures(rrt)
    # epase_idx, epase_success, epase_success_idx  = prune_failures(epase)

    
    common_idx = np.intersect1d(insat[:,0], pinsat[:,0])
    # common_idx = np.intersect1d(common_idx, rrt[:,0])
    # common_idx = np.intersect1d(common_idx, epase[:,0])
    

    insat_filtered = []
    pinsat_filtered = []
    rrt_filtered = []
    epase_filtered = []
    
    for idx in common_idx:
        insat_filtered.append(insat[np.where(insat[:,0] == idx)])
        pinsat_filtered.append(pinsat[np.where(pinsat[:,0] == idx)])
        # rrt_filtered.append(rrt[np.where(rrt[:,0] == idx)])
        # epase_filtered.append(epase[np.where(epase[:,0] == idx)])
    
    rrt_filtered = rrt
    epase_filtered = epase

    insat = np.array(insat_filtered).squeeze()
    pinsat = np.array(pinsat_filtered).squeeze()
    # rrt = np.array(rrt_filtered).squeeze()
    # epase = np.array(epase_filtered).squeeze()


    d['insat'] = {'id' : insat[:, 0], 'time' : insat[:, 1], 'cost' : insat[:, 2], 'length' : insat[:, 3], 'state_expansions' : insat[:, 4], 'edge_expansions' : insat[:, 5]} 
    d['pinsat'] = {'id' : pinsat[:, 0], 'time' : pinsat[:, 1], 'cost' : pinsat[:, 2], 'length' : pinsat[:, 3], 'state_expansions' : pinsat[:, 4], 'edge_expansions' : pinsat[:, 5]} 
    d['rrt'] = {'id' : rrt[:, 0], 'time' : rrt[:, 1], 'cost' : rrt[:, 2], 'length' : rrt[:, 3], 'state_expansions' : rrt[:, 4], 'edge_expansions' : rrt[:, 5]} 
    d['epase'] = {'id' : epase[:, 0], 'time' : epase[:, 1], 'cost' : epase[:, 2], 'length' : epase[:, 3], 'state_expansions' : epase[:, 4], 'edge_expansions' : epase[:, 5]} 


    # pdb.set_trace()


    # if not sanity_check(d, expansion_thresh):
    #     print("Sanity check failed")
    #     pdb.set_trace()



    stats = {'insat': {}, 'pinsat': {}, 'rrt': {}, 'epase': {}}


    stats['insat']['num_success_problems'] = d['insat']['id'].shape[0]
    stats['insat']['success_rate'] = insat_success
    stats['insat']['preprocess_success'] = insat_preprocess_success
    stats['insat']['mean_cost'] = np.mean(d['insat']['cost'])
    stats['insat']['std_cost'] = np.std(d['insat']['cost'])
    stats['insat']['mean_time'] = np.mean(d['insat']['time'])
    stats['insat']['std_time'] = np.std(d['insat']['time'])
    stats['insat']['median_time'] = np.median(d['insat']['time'])
    stats['insat']['max_time'] = np.max(d['insat']['time'])
    stats['insat']['mean_state_expansions'] = np.mean(d['insat']['state_expansions'])
    stats['insat']['median_state_expansions'] = np.median(d['insat']['state_expansions'])
    stats['insat']['mean_edge_expansions'] = np.mean(d['insat']['edge_expansions'])


    stats['pinsat']['num_success_problems'] = d['pinsat']['id'].shape[0]
    stats['pinsat']['success_rate'] = pinsat_success
    stats['pinsat']['preprocess_success'] = pinsat_preprocess_success
    stats['pinsat']['mean_cost'] = np.mean(d['pinsat']['cost'])
    stats['pinsat']['std_cost'] = np.std(d['pinsat']['cost'])
    stats['pinsat']['mean_time'] = np.mean(d['pinsat']['time'])
    stats['pinsat']['std_time'] = np.std(d['pinsat']['time'])
    stats['pinsat']['median_time'] = np.median(d['pinsat']['time'])
    stats['pinsat']['max_time'] = np.max(d['pinsat']['time'])
    stats['pinsat']['mean_state_expansions'] = np.mean(d['pinsat']['state_expansions'])
    stats['pinsat']['median_state_expansions'] = np.median(d['pinsat']['state_expansions'])    
    stats['pinsat']['mean_edge_expansions'] = np.mean(d['pinsat']['edge_expansions'])

    stats['rrt']['num_success_problems'] = d['rrt']['id'].shape[0]
    stats['rrt']['success_rate'] = rrt_success
    stats['rrt']['preprocess_success'] = rrt_preprocess_success
    stats['rrt']['mean_cost'] = np.mean(d['rrt']['cost'])
    stats['rrt']['std_cost'] = np.std(d['rrt']['cost'])
    stats['rrt']['mean_time'] = np.mean(d['rrt']['time'])
    stats['rrt']['std_time'] = np.std(d['rrt']['time'])
    stats['rrt']['median_time'] = np.median(d['rrt']['time'])
    stats['rrt']['max_time'] = np.max(d['rrt']['time'])
    stats['rrt']['mean_state_expansions'] = np.mean(d['rrt']['state_expansions'])
    stats['rrt']['mean_edge_expansions'] = np.mean(d['rrt']['edge_expansions'])

    stats['epase']['num_success_problems'] = d['epase']['id'].shape[0]
    stats['epase']['success_rate'] = epase_success
    stats['epase']['preprocess_success'] = epase_preprocess_success
    stats['epase']['mean_cost'] = np.mean(d['epase']['cost'])
    stats['epase']['std_cost'] = np.std(d['epase']['cost'])
    stats['epase']['mean_time'] = np.mean(d['epase']['time'])
    stats['epase']['std_time'] = np.std(d['epase']['time'])
    stats['epase']['median_time'] = np.median(d['epase']['time'])
    stats['epase']['max_time'] = np.max(d['epase']['time'])
    stats['epase']['mean_state_expansions'] = np.mean(d['epase']['state_expansions'])
    stats['epase']['mean_edge_expansions'] = np.mean(d['epase']['edge_expansions'])


    for alg in ['rrt', 'epase', 'insat', 'pinsat']:
        print("------------------")
        print(alg)
        print("------------------")
        for stat in np.sort(list(stats[alg])):
        # for stat in ['num_success_problems', 'success_rate', 'mean_time', 'std_time', 'mean_cost', 'std_cost']:
            # if stat in ['mean_time', 'std_time', 'mean_cost', 'std_cost']:
            print('{}: {}'.format(stat, np.round(stats[alg][stat],2)))
            # else:
            #     print('{}: {}'.format(stat, stats[alg][stat]))


    # pdb.set_trace()
    return stats



def get_intersecting_data(wastar, gepase, pase):

    # print("wastar: {} | pase: {} | gepase: {}".format(wastar.shape[0], pase.shape[0], gepase.shape[0]))    
    # pdb.set_trace()

    common_idx = np.intersect1d(wastar[:,0], gepase[:,0])
    common_idx = np.intersect1d(common_idx, pase[:, 0])

    wastar_filtered = []
    gepase_filtered = []
    pase_filtered = []

    for idx in common_idx:
        wastar_filtered.append(wastar[np.where(wastar[:,0] == idx)])
        gepase_filtered.append(gepase[np.where(gepase[:,0] == idx)])
        pase_filtered.append(pase[np.where(pase[:,0] == idx)])

    wastar = np.array(wastar_filtered).squeeze()
    gepase = np.array(gepase_filtered).squeeze()
    pase = np.array(pase_filtered).squeeze()

    return wastar, gepase, pase



if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--num_threads', '-nt', type=int, default=60)
    parser.add_argument('--dir', '-d', type=str, default="")

    args = parser.parse_args()
    
    base_dir = os.path.join("../logs", args.dir)


    insat = np.loadtxt(os.path.join(base_dir, "insat_smart_1.txt"))
    pinsat = np.loadtxt(os.path.join(base_dir, "pinsat_smart_" + str(args.num_threads) + ".txt"))
    rrt = np.loadtxt(os.path.join(base_dir, "rrtconnect_" + str(args.num_threads) + ".txt"))
    epase = np.loadtxt(os.path.join(base_dir, "gepase_" + str(args.num_threads) + ".txt"))

    pdb.set_trace()    

    np.set_printoptions(suppress=True)

    stats = {}
    stats[0] = print_stats(insat, pinsat, rrt, epase, 0, 20)

