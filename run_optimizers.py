import pickle
import random
from ortools.linear_solver import pywraplp
import numpy as np
import matplotlib.pyplot as plt
random.seed(123)

BIG_M = 10e6


def create_variables(points):
    
    solver = pywraplp.Solver('simple_mip_program',
                             pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
    
    y_var_dict = {}
    t_var_dict = {}
    
    # Define trip of truck_i from point_j to point_k
    for t in range(N_TRUCKS):
        for p_j in points:
            for p_k in points:
                if p_j != p_k:
                    y_var_dict[(t, p_j, p_k)] =  solver.BoolVar(f'y_{t}_{p_j}_{p_k}')

    for t in range(N_TRUCKS):
        for p_j in points:
                t_var_dict[(t, p_j)] =  solver.IntVar(0, BIG_M, f't_{t}_{p_j}')

    return solver, y_var_dict, t_var_dict


def capacity_per_truck_constraint_trips(solver, y_var_dict, points, demands_dict):
    """
    Limits the capacity per truck
    
    """
    capacity = int(CAPACITY_PER_TRUCK)
    for t in range(N_TRUCKS):
        ct = solver.Constraint(0, capacity, f'Capacity_Truck_{t}')
        for p_j in points:
            for p_k in points:
                if p_j != p_k:
                    ct.SetCoefficient(y_var_dict[(t, p_j, p_k)], demands_dict[p_k])
    return solver

"""
def visit_all_points(solver, x_var_dict, points):
    # A truck must visit all points
    for p in points:
        if p != 0:
            ct = solver.Constraint(1, 1, f'Visit_point_{p}')
            for t in range(N_TRUCKS):
                ct.SetCoefficient(x_var_dict[(t, p)], 1)
    
    # Visit hub twice
    for t in range(N_TRUCKS):
        ct = solver.Constraint(2, 2, f'Truck_{t}_visits_hub_twice')
        ct.SetCoefficient(x_var_dict[(t, 0)], 1)
    return solver
|"""

def travel_from_all_points(solver, y_var_dict, points):
    # Makes sure that a trip has to start from all points
    for p in points:
        if p != 0:
            ct = solver.Constraint(1, 1, f'Travel_from_point_{p}')
            for t in range(N_TRUCKS):
                for p_j in points:
                    if p != p_j:
                        ct.SetCoefficient(y_var_dict[(t, p, p_j)], 1)
        else:
            # From the hub, N_TRUCKS trips must start
            ct = solver.Constraint(N_TRUCKS, N_TRUCKS, f'Travel_from_point_{p}')
            for t in range(N_TRUCKS):
                for p_j in points:
                    if p != p_j:
                        ct.SetCoefficient(y_var_dict[(t, p, p_j)], 1)
    return solver
                        
def travel_to_all_points(solver, y_var_dict, points):
    # Makes sure that a trip has to start from all points
    for p in points:
        if p != 0:
            ct = solver.Constraint(1, 1, f'Travel_to_point_{p}')
            for t in range(N_TRUCKS):
                for p_j in points:
                    if p != p_j:
                        ct.SetCoefficient(y_var_dict[(t, p_j, p)], 1)
        else:
            # From the hub, N_TRUCKS trips must start
            ct = solver.Constraint(N_TRUCKS, N_TRUCKS, f'Travel_to_point_{p}')
            for t in range(N_TRUCKS):
                for p_j in points:
                    if p != p_j:
                        ct.SetCoefficient(y_var_dict[(t, p_j, p)], 1)
                        
    return solver

def flow_conserve(solver, y_var_dict, points):
    # Makes sure that a trip has to start from all points
    for t in range(N_TRUCKS):
        for p in points:
            ct = solver.Constraint(0, 0, f'FlowConserve_{t}_{p}')
            for p_j in points:
                if p != p_j:
                    ct.SetCoefficient(y_var_dict[(t, p_j, p)], 1)
                    ct.SetCoefficient(y_var_dict[(t, p, p_j)], -1)
                        
    return solver

def avoid_subtours(y_var_dict, t_var_dict, points, solver):
    
    checked_tuples = []
    """
    for t in range(N_TRUCKS):
        for p_i in points:
            for p_j in points:
                if p_i != p_j and p_i != 0:
                    if (p_i, p_j, t) not in checked_tuples:
                        ct = solver.Constraint(-solver.infinity(), BIG_M - 1, f'Avoid_SubTour_{t}_{p_i}_{p_j}')
                        ct.SetCoefficient(t_var_dict[(t, p_i)], 1)
                        ct.SetCoefficient(t_var_dict[(t, p_j)], -1)
                        ct.SetCoefficient(y_var_dict[(t, p_i, p_j)], BIG_M)
                        checked_tuples.append((p_i, p_j, t))
    """                 
    for t in range(N_TRUCKS):
        for p_i in points:
            for p_j in points:
                if p_i != p_j and p_i != 0:
                    if (p_i, p_j, t) not in checked_tuples:
                        ct = solver.Constraint(-solver.infinity(), BIG_M - 1, f'Avoid_SubTour_{t}_{p_i}_{p_j}')
                        ct.SetCoefficient(t_var_dict[(t, p_i)], 1)
                        ct.SetCoefficient(t_var_dict[(t, p_j)], -1)
                        ct.SetCoefficient(y_var_dict[(t, p_i, p_j)], BIG_M)

                        #row = [ind, val]
                        #solver.linear_constraints.add(lin_expr=[row], senses=['L'], rhs=[BIG_M - 1])
                        
    return solver            


def all_trucks_should_visit_depot_twice(y_var_dict, points, solver):
    for t in range(N_TRUCKS):
        ct = solver.Constraint(1, 1, f'FromDepot_SubTour_{t}')
        for p in points:
            if p != 0:
                ct.SetCoefficient(y_var_dict[(t, 0, p)], 1)
                    
    for t in range(N_TRUCKS):
        ct = solver.Constraint(1, 1, f'ToDepot_SubTour_{t}')
        for p in points:
            if p != 0:
                ct.SetCoefficient(y_var_dict[(t, p, 0)], 1)
    return solver


def add_constraint_matrix(y_var_dict, t_var_dict, points, demands_dict, solver):
    solver = capacity_per_truck_constraint_trips(solver, y_var_dict, points, demands_dict)
    solver = travel_from_all_points(solver, y_var_dict, points)
    solver = travel_to_all_points(solver, y_var_dict, points)
    solver = flow_conserve(solver, y_var_dict, points)
    solver = avoid_subtours(y_var_dict, t_var_dict, points, solver)
    solver = all_trucks_should_visit_depot_twice(y_var_dict, points, solver)
    return solver

def set_obj_function(y_var_dict, t_var_dict, points, dist_matrix, solver):
    objective = solver.Objective()
    for t in range(N_TRUCKS):
        for p_i in points:
            for p_j in points:
                if p_i != p_j:
                    objective.SetCoefficient(y_var_dict[(t, p_i, p_j)], dist_matrix[(p_i, p_j)])
    objective.SetMinimization()
    with open("model.lp", "w") as out_f:
        lp_text = solver.ExportModelAsLpFormat(obfuscated=False)
        out_f.write(lp_text)
        
    solver.Solve()
    return solver, y_var_dict, objective

def get_trips_by_truck(y_var_dict, t_var_dict, points):
    solution_dict = {}
    for t in range(N_TRUCKS):
        for p_i in points:
            for p_j in points:
                if p_i != p_j:
                    if round(y_var_dict[(t, p_i, p_j)].solution_value()) == 1:
                        print(t, p_i, p_j)            
    
    for t in range(N_TRUCKS):
        print(t)
        solution_dict[t] = [0]
        from_station = 0
        to_station = 1
        evaluated_points = [0]
        while len(evaluated_points) != len(points):
            for p in points:
                try:
                    if round(y_var_dict[(t, from_station, p)].solution_value()) == 1:
                        solution_dict[t].append(p)
                        to_station = p
                        from_station = p
                        evaluated_points.append(p)
                except:
                    pass
                        
    plt.figure(figsize=(8,8))
    for i in points:    
        if i == 0:
            plt.scatter(points[i]['x'], points[i]['y'], c='green', s=200)
        else:
            plt.scatter(points[i]['x'], points[i]['y'], c='orange', s=200)

    for k in range(N_TRUCKS):
        for i in points:
            for j in points:
                if i != j and round(y_var_dict[(k, i, j)].solution_value()) == 1:
                    plt.plot([points[i]['x'], points[j]['x']], [points[i]['y'], points[j]['y']], c="black")

    plt.show()
    points
    return solution_dict
            
        
if __name__ == '__main__':
    
    for N_POINTS in [10, 20, 50]:
        # Open information
        with open(f'./data/points_{N_POINTS}.pickle', 'rb') as handle:
            points = pickle.load(handle)

        with open(f'./data/dist_matrix_{N_POINTS}.pickle', 'rb') as handle:
            dist_matrix = pickle.load(handle)

        for trucks in [5]:
            for diff_mode in ['high', 'low']:                
                # Set number of trucks
                global N_TRUCKS
                N_TRUCKS = trucks
                
                # Open demand and capacities info                
                with open(f'./data/demand_{N_POINTS}_{diff_mode}.pickle', 'rb') as handle:
                    all_demands_dict = pickle.load(handle)
                    
                with open(f'./data/capacities_{N_POINTS}_{diff_mode}.pickle', 'rb') as handle:
                    capacities_dict = pickle.load(handle)

                if N_TRUCKS == 1:
                    capacity = capacities_dict['max_demand']
                else:
                    capacity = capacities_dict['avg_demand']
                global CAPACITY_PER_TRUCK
                CAPACITY_PER_TRUCK = capacity

                for n in range(1):
                    demands_dict = all_demands_dict[n]
                    
                    # Create variable
                    solver, y_var_dict, t_var_dict = create_variables(points)
                    
                    
                    # Create constraint matrix
                    solver = add_constraint_matrix(y_var_dict, t_var_dict, points, demands_dict, solver)
                    
                    # Add objective function and solve
                    solver, y_var_dict, objective = set_obj_function(y_var_dict, t_var_dict, points, dist_matrix, solver)
                    
                    # Get solution
                    solution_dict = get_trips_by_truck(y_var_dict, t_var_dict, points)
                    
                    sols = []
                    for t in solution_dict:
                        sols = sols + solution_dict[t]
                    sols = list(set(sols))
                    a = len(sols)