import time
from typing import Iterator
from ortools.linear_solver import pywraplp

from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.route import Route
from vehicle_routing_problem.metaheuristics.base_metaheuristic import BaseMetaheuristic

class LinearProgrammingSolver(BaseMetaheuristic):
    """
    Adapter pour résoudre le VRPTW via Programmation Linéaire (OR-Tools SCIP).
    Implémente les contraintes C1 à C7 du document 'Projet.pdf'.
    """

    def __init__(self, instance: Instance, time_limit: int = 60):
        super().__init__(instance)
        self.time_limit = time_limit

    def solve(self, current_solution: Solution = None) -> Iterator[Solution]:
        """
        Transforme l'Instance en modèle PLNE, résout via Simplexe/Dakin, 
        et yield une Solution compatible avec ton Visualizer.
        """
        solver = pywraplp.Solver.CreateSolver('SCIP')
        if not solver:
            return

        solver.SetTimeLimit(self.time_limit * 1000)

        # Raccourcis pour la lisibilité (N=clients+dépôt, K=véhicules)
        N = list(range(len(self._inst.clients)))
        # On estime une borne sup du nb de véhicules (1 par client)
        K = list(range(len(self._inst.clients))) 
        dist = self._inst.dist_matrix

        # --- 1. VARIABLES DE DÉCISION ---
        # x[i,j,k] : Variable binaire de routage
        x = {}
        for i in N:
            for j in N:
                if i != j:
                    for k in K:
                        x[i, j, k] = solver.BoolVar(f'x_{i}_{j}_{k}')

        # T[i,k] : Variable continue pour le temps d'arrivée
        T = {}
        for i in N:
            for k in K:
                T[i, k] = solver.NumVar(0, 10000, f'T_{i}_{k}')

        # --- 2. FONCTION OBJECTIF (Distance totale) ---
        obj = solver.Objective()
        for i in N:
            for j in N:
                if i != j:
                    for k in K:
                        obj.SetCoefficient(x[i, j, k], dist[i][j])
        obj.SetMinimization()

        # --- 3. CONTRAINTES (Modélisation Projet.pdf) ---
        # C1 : Chaque client visité 1 seule fois[cite: 2]
        for i in N[1:]:
            ct = solver.Constraint(1, 1)
            for k in K:
                for j in N:
                    if i != j:
                        ct.SetCoefficient(x[i, j, k], 1)

        # C2 : Conservation du flot[cite: 2]
        for i in N[1:]:
            for k in K:
                ct = solver.Constraint(0, 0)
                for j in N:
                    if i != j:
                        ct.SetCoefficient(x[i, j, k], 1)
                        ct.SetCoefficient(x[j, i, k], -1)

        # C3 & C4 : Départ et Retour au dépôt[cite: 2]
        for k in K:
            solver.Add(sum(x[0, j, k] for j in N if j != 0) <= 1)
            solver.Add(sum(x[i, 0, k] for i in N if i != 0) <= 1)

        # C5 : Capacité[cite: 2]
        for k in K:
            solver.Add(sum(self._inst.clients[i].demand * x[i, j, k] 
                           for i in N[1:] for j in N if i != j) <= self._inst.capacity)

        # C6 & C7 : Fenêtres de temps et Big-M serré[cite: 2]
        for i in N:
            ci = self._inst.clients[i]
            for k in K:
                solver.Add(T[i, k] >= ci.ready_time)
                solver.Add(T[i, k] <= ci.due_time)

            for j in N[1:]:
                if i != j:
                    # M_ij serré = b_i + s_i + t_ij - a_j[cite: 2]
                    M_ij = ci.due_time + ci.service_time + dist[i][j] - self._inst.clients[j].ready_time
                    for k in K:
                        solver.Add(T[j, k] >= T[i, k] + ci.service_time + dist[i][j] - M_ij * (1 - x[i, j, k]))

        # --- 4. RÉSOLUTION ---
        status = solver.Solve()

        if status in [pywraplp.Solver.OPTIMAL, pywraplp.Solver.FEASIBLE]:
            # On reconstruit tes objets Route et Solution
            routes_list = []
            for k in K:
                route_nodes = []
                curr = 0
                while True:
                    next_node = None
                    for j in N:
                        if i != j and x.get((curr, j, k)) and x[curr, j, k].solution_value() > 0.5:
                            next_node = j
                            break
                    if next_node is None or next_node == 0:
                        break
                    route_nodes.append(next_node)
                    curr = next_node
                
                # Création de ton objet Route (assumé : prend instance et liste d'IDs)
                routes_list.append(Route(self._inst, route_nodes))
            
            yield Solution(routes_list)