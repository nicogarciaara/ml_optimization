import random
import pickle
import math
import numpy as np
from tqdm import tqdm
random.seed(123)


def define_points(N_POINTS):
    points = {}
    coordinates_list = []
    # Define coordinates
    for i in range(N_POINTS):
        check = 0
        while check == 0:
            coordinates = (random.randint(1, 50), random.randint(1, 50))
            if coordinates not in coordinates_list:
                points[i] = {'x': coordinates[0], 'y': coordinates[1]}
                coordinates_list.append(coordinates)
                check = 1
    return points

def define_distance_matrix(points):
    dist_matrix = {}
    for p_i in points:
        for p_j in points:
            distance = math.sqrt((points[p_i]['x'] - points[p_j]['x'])**2 +(points[p_i]['y'] - points[p_j]['y'])**2)
            dist_matrix[(p_i, p_j)] = distance
    return dist_matrix

def create_demands_per_station(points, diff_mode):
    demands_dict = {}
    for p in points:
        if p == 0:
            demands_dict[p] = 0
        else:
            if diff_mode == 'high':
                demands_dict[p] = random.randint(1, 50)
            else:
                demands_dict[p] = random.randint(40, 50)
    return demands_dict


if __name__ == '__main__':
    for N_POINTS in [10, 12, 15, 18, 20, 50]:
        points = define_points(N_POINTS)
        dist_matrix = define_distance_matrix(points)
        
        with open(f'./data/points_{N_POINTS}.pickle', 'wb') as handle:
            pickle.dump(points, handle, protocol=pickle.HIGHEST_PROTOCOL)
            
        with open(f'./data/dist_matrix_{N_POINTS}.pickle', 'wb') as handle:
            pickle.dump(dist_matrix, handle, protocol=pickle.HIGHEST_PROTOCOL)
            
        for diff_mode in ['high', 'low']:
            # Simulate demand
            demand_dict_all = {}
            max_demand = 0
            for i in tqdm(range(250000)):
                demands_dict = create_demands_per_station(points, diff_mode)
                demand = np.sum([demands_dict[p] for p in demands_dict.keys()])
                if demand > max_demand:
                    max_demand = demand
                demand_dict_all[i] = demands_dict

            with open(f'./data/demand_{N_POINTS}_{diff_mode}.pickle', 'wb') as handle:
                pickle.dump(demand_dict_all, handle, protocol=pickle.HIGHEST_PROTOCOL)
            
            for c in [0, 0.1, 0.2, 0.5, 0.8]:
                capacities_dict = {
                    'max_demand': max_demand,
                    'avg_demand': np.ceil(max_demand* (1 + c) / 5)
                }
                
                with open(f'./data/capacities_{N_POINTS}_{diff_mode}_{c}.pickle', 'wb') as handle:
                    pickle.dump(capacities_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
            
        
