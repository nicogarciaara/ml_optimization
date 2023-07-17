import pickle
import random
from ortools.linear_solver import pywraplp
import numpy as np
import cplex
import matplotlib.pyplot as plt
import time
import pandas as pd
from tqdm import tqdm

random.seed(123)

BIG_M = 10e6


def create_variables(points):
    
    solver = cplex.Cplex()
    
    y_var_dict = {}
    t_var_dict = {}
    
    n = 0
    
    # Define trip of truck_i from point_j to point_k
    for t in range(N_TRUCKS):
        for p_j in points:
            for p_k in points:
                if p_j != p_k:
                    y_var_dict[(t, p_j, p_k)] =  n
                    n += 1

    for t in range(N_TRUCKS):
        for p_j in points:
                t_var_dict[(t, p_j)] =  n
                n += 1

    return solver, y_var_dict, t_var_dict


def capacity_per_truck_constraint_trips(solver, y_var_dict, points, demands_dict):
    """
    Limits the capacity per truck
    
    """
    capacity = int(CAPACITY_PER_TRUCK)
    for t in range(N_TRUCKS):
        #ct = solver.Constraint(0, capacity, f'Capacity_Truck_{t}')
        ind = []
        val = []
        for p_j in points:
            for p_k in points:
                if p_j != p_k:
                    #ct.SetCoefficient(y_var_dict[(t, p_j, p_k)], demands_dict[p_k])
                    ind.append(y_var_dict[(t, p_j, p_k)])
                    val.append(demands_dict[p_k])
        row = [ind, val]
        solver.linear_constraints.add(lin_expr=[row], senses=['L'], rhs=[capacity])
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
            #ct = solver.Constraint(1, 1, f'Travel_from_point_{p}')
            ind = []
            val = []
            for t in range(N_TRUCKS):
                for p_j in points:
                    if p != p_j:
                        #ct.SetCoefficient(y_var_dict[(t, p, p_j)], 1)
                        ind.append(y_var_dict[(t, p, p_j)])
                        val.append(1)
            row = [ind, val]
            solver.linear_constraints.add(lin_expr=[row], senses=['E'], rhs=[1])
        else:
            # From the hub, N_TRUCKS trips must start
            #ct = solver.Constraint(N_TRUCKS, N_TRUCKS, f'Travel_from_point_{p}')
            ind = []
            val = []
            for t in range(N_TRUCKS):
                for p_j in points:
                    if p != p_j:
                        #ct.SetCoefficient(y_var_dict[(t, p, p_j)], 1)
                        ind.append(y_var_dict[(t, p, p_j)])
                        val.append(1)
            row = [ind, val]
            solver.linear_constraints.add(lin_expr=[row], senses=['L'], rhs=[N_TRUCKS])
    return solver
                        
def travel_to_all_points(solver, y_var_dict, points):
    # Makes sure that a trip has to start from all points
    for p in points:
        if p != 0:
            #ct = solver.Constraint(1, 1, f'Travel_to_point_{p}')
            ind = []
            val = []
            for t in range(N_TRUCKS):
                for p_j in points:
                    if p != p_j:
                        #ct.SetCoefficient(y_var_dict[(t, p_j, p)], 1)
                        ind.append(y_var_dict[(t, p_j, p)])
                        val.append(1)
            row = [ind, val]
            solver.linear_constraints.add(lin_expr=[row], senses=['E'], rhs=[1])
        else:
            # From the hub, N_TRUCKS trips must start
            #ct = solver.Constraint(N_TRUCKS, N_TRUCKS, f'Travel_to_point_{p}')
            ind = []
            val = []
            for t in range(N_TRUCKS):
                for p_j in points:
                    if p != p_j:
                        #ct.SetCoefficient(y_var_dict[(t, p_j, p)], 1)
                        ind.append(y_var_dict[(t, p_j, p)])
                        val.append(1)
            row = [ind, val]
            solver.linear_constraints.add(lin_expr=[row], senses=['L'], rhs=[N_TRUCKS])
                        
    return solver

def flow_conserve(solver, y_var_dict, points):
    # Makes sure that a trip has to start from all points
    for t in range(N_TRUCKS):
        for p in points:
            #ct = solver.Constraint(0, 0, f'FlowConserve_{t}_{p}')
            ind = []
            val = []
            for p_j in points:
                if p != p_j:
                    #ct.SetCoefficient(y_var_dict[(t, p_j, p)], 1)
                    #ct.SetCoefficient(y_var_dict[(t, p, p_j)], -1)
                    ind.append(y_var_dict[(t, p_j, p)])
                    val.append(1)
                    ind.append(y_var_dict[(t, p, p_j)])
                    val.append(-1)
            row = [ind, val]
            solver.linear_constraints.add(lin_expr=[row], senses=['E'], rhs=[0])
                        
    return solver

def avoid_subtours(y_var_dict, t_var_dict, points, solver):
    checked_tuples = []
    for t in range(N_TRUCKS):
        for p_i in points:
            for p_j in points:
                if p_i != p_j and p_i != 0:
                    if (p_i, p_j, t) not in checked_tuples:
                        #ct = solver.Constraint(-BIG_M, BIG_M - 1, f'Avoid_SubTour_{t}_{p_i}_{p_j}')
                        #ct.SetCoefficient(t_var_dict[(t, p_i)], 1)
                        #ct.SetCoefficient(t_var_dict[(t, p_j)], -1)
                        #ct.SetCoefficient(y_var_dict[(t, p_i, p_j)], BIG_M)
                        ind = []
                        val = []
                        ind.append(t_var_dict[(t, p_i)])
                        ind.append(t_var_dict[(t, p_j)])
                        ind.append(y_var_dict[(t, p_i, p_j)])
                        val.append(1)
                        val.append(-1)
                        val.append(BIG_M)
                        row = [ind, val]
                        solver.linear_constraints.add(lin_expr=[row], senses=['L'], rhs=[BIG_M - 1])
    return solver

            


def all_trucks_should_visit_depot_twice(y_var_dict, points, solver):
    
    for t in range(N_TRUCKS):
        #ct = solver.Constraint(1, 1, f'FromDepot_SubTour_{t}')
        ind = []
        val = []
        for p in points:
            if p != 0:
                #ct.SetCoefficient(y_var_dict[(t, 0, p)], 1)
                ind.append(y_var_dict[(t, 0, p)])
                val.append(1)
        row = [ind, val]
        solver.linear_constraints.add(lin_expr=[row], senses=['L'], rhs=[1])
                    
    for t in range(N_TRUCKS):
        ind = []
        val = []
        #ct = solver.Constraint(1, 1, f'ToDepot_SubTour_{t}')
        for p in points:
            if p != 0:
                #ct.SetCoefficient(y_var_dict[(t, p, 0)], 1)
                ind.append(y_var_dict[(t, p, 0)])
                val.append(1)
        row = [ind, val]
        solver.linear_constraints.add(lin_expr=[row], senses=['L'], rhs=[1])
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
    """
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
        
    solver.set_time_limit = 30
    solver.Solve()
    """
    coef = []
    lower_bounds = []
    upper_bounds = []
    types = []
    names = []
    
    # Define trip of truck_i from point_j to point_k
    for t in range(N_TRUCKS):
        for p_j in points:
            for p_k in points:
                if p_j != p_k:
                    coef.append(dist_matrix[(p_j, p_k)])
                    lower_bounds.append(0)
                    upper_bounds.append(1)
                    types.append('B')
                    names.append(f'y_{t}_{p_j}_{p_k}')
                    

    for t in range(N_TRUCKS):
        for p_j in points:
                coef.append(0)
                lower_bounds.append(0)
                upper_bounds.append(len(points) + 1)
                types.append('I')
                names.append(f't_{t}_{p_j}')
    
    solver.variables.add(obj=coef, lb=lower_bounds, ub=upper_bounds, types=types, names=names)
    solver.objective.set_sense(solver.objective.sense.minimize)
    solver = add_constraint_matrix(y_var_dict, t_var_dict, points, demands_dict, solver)
    solver.write(f"C:/Users/HP/Documents/Analisis Varios/ML + Optimization/cplexmodel.lp")
    start = time.time()
    #solver.parameters.timelimit.set(10)
    solver.solve()
    end = time.time()
    time_taken = end - start
    
    y_var_dict_solve = solver.solution.get_values()
    objective = solver.solution.get_objective_value()

    return solver, y_var_dict, y_var_dict_solve, objective, time_taken

def get_trips_by_truck(N_POINTS, diff_mode, y_var_dict, y_var_dict_solve, t_var_dict, points, objective, time_taken, demands_dict):
    solution_dict = {}
    # Print solution
    for t in range(N_TRUCKS):
        for p_i in points:
            for p_j in points:
                if p_i != p_j:
                    idx = y_var_dict[(t, p_i, p_j)]
                    if round(y_var_dict_solve[idx]) == 1:
                        print(t, p_i, p_j)
    
    # Get truck order -> truck number 0 will be the 1 that goes to the "lowest" point
    first_point = []
    trucks_list = []
    for t in range(N_TRUCKS):
        for p in points:
            if p != 0:
                idx = y_var_dict[(t, 0, p)]
                if round(y_var_dict_solve[idx]) == 1:
                    first_point.append(p)
                    trucks_list.append(t)
    trucks_df = pd.DataFrame(data={
        'Truck': trucks_list,
        'First': first_point
    }).sort_values(by='First').reset_index(drop=True)
    trucks_df['Order'] = trucks_df.index
    
    # Make sure we include all info
    not_selected_trucks = [x for x in range(N_TRUCKS) if x not in trucks_list]
    if len(not_selected_trucks) != 0:
        trucks_df_aux = pd.DataFrame(data={
            'Truck': not_selected_trucks
        })
        trucks_df_aux['First'] = 99
        trucks_df_aux['Order'] = 99
        trucks_df = pd.concat([trucks_df, trucks_df_aux], ignore_index=True)
    
    # Create output dataframe
    output_df = pd.DataFrame(data={
        'n_points': [N_POINTS],
        'n_trucks': [N_TRUCKS],
        'diff_mode': [diff_mode],
        'distance': [objective],
        'Time': [time_taken],
    })
        
    # Generate demand as columns
    for p in points:
        output_df[f'demand_{p}'] = demands_dict[p]
        
    
    # Iterate over solution to create log
    for index, row in trucks_df.iterrows():
        output_key = row['Order']
        solution_key = row['Truck']
        for p_i in points:
            for p_j in points:
                if p_i != p_j:
                    idx = y_var_dict[(solution_key, p_i, p_j)]
                    if round(y_var_dict_solve[idx]) == 1:
                        output_df[f'solution_{output_key}_{p_i}_{p_j}'] = 1
                    else:
                        output_df[f'solution_{output_key}_{p_i}_{p_j}'] = 0
    
    # Plot solution
    """
    plt.figure(figsize=(8,8))
    for i in points:    
        if i == 0:
            plt.scatter(points[i]['x'], points[i]['y'], c='green', s=200)
        else:
            plt.scatter(points[i]['x'], points[i]['y'], c='orange', s=200)

    for k in range(N_TRUCKS):
        for i in points:
            for j in points:
                try:
                    idx = y_var_dict[(k, i, j)]
                    if i != j and round(y_var_dict_solve[idx]) == 1:
                        plt.plot([points[i]['x'], points[j]['x']], [points[i]['y'], points[j]['y']], c="black")
                except:
                    pass
    plt.show()   
    """
    return output_df
            
        
if __name__ == '__main__':
    
    #for N_POINTS in [12, 15, 18]:
    for N_POINTS in [12, 15]:
        # Open information
        with open(f'./data/points_{N_POINTS}.pickle', 'rb') as handle:
            points = pickle.load(handle)

        with open(f'./data/dist_matrix_{N_POINTS}.pickle', 'rb') as handle:
            dist_matrix = pickle.load(handle)

        for trucks in [3, 5]:
            for diff_mode in ['high', 'low']:                
                # Set number of trucks
                global N_TRUCKS
                N_TRUCKS = trucks
                
                # Open demand and capacities info                
                with open(f'./data/demand_{N_POINTS}_{diff_mode}.pickle', 'rb') as handle:
                    all_demands_dict = pickle.load(handle)
                    
                with open(f'./data/capacities_{N_POINTS}_{diff_mode}.pickle', 'rb') as handle:
                    capacities_dict = pickle.load(handle)
                
                for cap_overwrite in [0, 0.25, 0.5]:        
                    capacity = np.ceil(capacities_dict['max_demand']*(1 + cap_overwrite)/trucks)
                    global CAPACITY_PER_TRUCK
                    CAPACITY_PER_TRUCK = capacity

                    output_df_all = pd.DataFrame()
                    for n in tqdm(range(11000)):
                        demands_dict = all_demands_dict[n]
                        
                        # Create variable
                        solver, y_var_dict, t_var_dict = create_variables(points)
                        
                                            
                        # Add objective function and solve
                        try:
                            solver, y_var_dict, y_var_dict_solve, objective, time_taken = set_obj_function(y_var_dict, t_var_dict, points, dist_matrix, solver)
                            
                            # Get solution
                            output_df = get_trips_by_truck(N_POINTS, diff_mode, y_var_dict, y_var_dict_solve, t_var_dict, points, objective, time_taken, demands_dict)
                            output_df_all = pd.concat([output_df_all, output_df], ignore_index=True)
                        except:
                            pass
                    
                    output_df_all.to_csv(f'./results/optimal_model_{N_POINTS}_{trucks}_{diff_mode}_{cap_overwrite}.csv', index=False)
                        
