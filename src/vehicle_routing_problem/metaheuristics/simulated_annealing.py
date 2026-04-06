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
    Recuit Simulé : explore TOUS les voisinages disponibles, choix aléatoire
    parmi les voisins avec probabilité Metropolis.
    """

    def __init__(self, instance: Instance, initial_temperature: float, cooling_rate: float):
        super().__init__(instance)
        self._initial_temperature = initial_temperature
        self._cooling_rate = cooling_rate

    def solve(self, current_solution: Solution) -> Iterator[Solution]:
        current = current_solution.copy()
        temperature = self._initial_temperature
        self.current_temperature = temperature

        while temperature > 1e-6:
            # Récupère TOUS les opérateurs disponibles
            all_operators = BaseOperator.get_operators(self._inst)

            # Génère TOUS les voisins possibles
            all_neighbors: list[BaseOperator] = []
            for op_type in all_operators:
                all_neighbors.extend(op_type.generate_neighbors(self._inst, current))

            if not all_neighbors:
                break

            # Choix aléatoire d'un voisin (simple !)
            candidate_op = random.choice(all_neighbors)
            candidate = candidate_op.apply(current)  # apply() fait déjà le copy()

            delta = candidate.total_distance - current.total_distance

            # Critère de Metropolis : accepte toujours les améliorations
            if delta < 0 or random.random() < math.exp(-delta / temperature):
                current = candidate

            # Refroidissement géométrique
            temperature *= self._cooling_rate
            self.current_temperature = temperature
            yield current
