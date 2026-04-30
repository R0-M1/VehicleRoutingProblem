from __future__ import annotations
import math
import random
from typing import Iterator

from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.metaheuristics.base_metaheuristic import BaseMetaheuristic
from vehicle_routing_problem.operators.base_operator import BaseOperator


class SimulatedAnnealing(BaseMetaheuristic):
    """
    Recuit Simulé avec :
    - n_neighbors voisins évalués par itération (on garde le meilleur)
    - Critère Metropolis sur le delta du meilleur voisin
    - Suivi de la meilleure solution globale (best_solution)
    - Le refroidissement n'avance QUE si un voisin a été trouvé
    """

    def __init__(
        self,
        instance: Instance,
        initial_temperature: float,
        cooling_rate: float,
        n_neighbors: int = 1000,
        check_time_windows: bool = True,
    ):
        super().__init__(instance)
        self._initial_temperature = initial_temperature
        self._cooling_rate = cooling_rate
        self._n_neighbors = n_neighbors
        self._check_time_windows = check_time_windows
        self.observers = []
        self.current_temperature = initial_temperature

    def add_observer(self, observer):
        self.observers.append(observer)

    def _is_feasible(self, solution: Solution, affected: list[int]) -> bool:
        for i in affected:
            route = solution.routes[i]
            if not route.is_capacity_feasible:
                return False
            if self._check_time_windows and not route.is_time_feasible():
                return False
        return True

    def solve(self, current_solution: Solution) -> Iterator[Solution]:
        current = current_solution.copy()
        best = current.copy()
        best_cost = best.total_distance

        temperature = self._initial_temperature
        self.current_temperature = temperature
        all_operators = BaseOperator.get_operators(self._inst)

        while temperature > 1e-6:
            # Échantillonnage de n_neighbors voisins valides
            best_op = None
            best_delta = float("inf")
            attempts = 0
            max_attempts = self._n_neighbors * 20
            found = 0

            while found < self._n_neighbors and attempts < max_attempts:
                attempts += 1
                op_type = random.choice(all_operators)
                candidate_op = op_type.sample_random_neighbor(self._inst, current)
                if candidate_op is None:
                    continue

                delta = candidate_op.get_delta_cost(current)

                if delta >= best_delta:
                    found += 1
                    continue

                candidate = candidate_op.apply(current)
                if not self._is_feasible(candidate, candidate_op.affected_routes()):
                    found += 1
                    continue

                found += 1
                best_delta = delta
                best_op = candidate_op

            # Pas de voisin trouvé : on skip sans refroidir
            if best_op is None:
                continue

            # --- Critère Metropolis sur le meilleur voisin ---
            if best_delta < 0 or random.random() < math.exp(-best_delta / temperature):
                current = best_op.apply(current)

                # --- Mise à jour de la meilleure solution globale ---
                current_cost = current.total_distance
                if current_cost < best_cost:
                    best = current.copy()
                    best_cost = current_cost

            # Refroidissement géométrique
            temperature *= self._cooling_rate
            self.current_temperature = temperature

            yield best