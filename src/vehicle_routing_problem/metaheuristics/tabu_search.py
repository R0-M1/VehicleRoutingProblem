from __future__ import annotations
from collections import deque
import random
from typing import Iterator

from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.metaheuristics.base_metaheuristic import BaseMetaheuristic
from vehicle_routing_problem.operators.base_operator import BaseOperator


class TabuSearch(BaseMetaheuristic):
    """
    Recherche Tabou : explore TOUS les voisinages disponibles et choisit
    le meilleur mouvement non tabou.
    """

    def __init__(self, instance: Instance, tabu_size: int, n_neighbors: int = 100):
        super().__init__(instance)
        self._tabu_size = tabu_size
        self._n_neighbors = n_neighbors
        self.tabu_list: deque[tuple] = deque(maxlen=self._tabu_size)
        self.tabu_set: set[tuple] = set()

    def _is_tabu(self, op: BaseOperator) -> bool:
        """Vérification en O(1) grâce au set et à la signature."""
        return op.get_signature() in self.tabu_set

    def _add_tabu(self, op: BaseOperator):
        """Ajoute un opérateur à la liste et au set tabou."""
        signature = op.get_signature()
        if len(self.tabu_list) == self._tabu_size:
            old_signature = self.tabu_list.popleft()
            self.tabu_set.remove(old_signature)
        self.tabu_list.append(signature)
        self.tabu_set.add(signature)

    def solve(self, current_solution: Solution) -> Iterator[Solution]:
        self.tabu_list.clear()
        self.tabu_set.clear()
        current = current_solution.copy()

        while True:
            all_operators = BaseOperator.get_operators(self._inst)
            
            sampled_neighbors = []
            available_ops = list(all_operators)

            attempts = 0
            max_attempts = self._n_neighbors * 10  # Sécurité pour éviter boucle infinie
            
            while len(sampled_neighbors) < self._n_neighbors and available_ops and attempts < max_attempts:
                attempts += 1
                op_type = random.choice(available_ops)
                
                neighbor = op_type.sample_random_neighbor(self._inst, current)
                
                if neighbor is not None:
                    sampled_neighbors.append(neighbor)
                else:
                    # Si l'opérateur retourne None (impossible, ex: 1 seule route dispo pour un InterExchange), on l'abandonne
                    available_ops.remove(op_type)

            if not sampled_neighbors:
                break

            non_tabu = [op for op in sampled_neighbors if not self._is_tabu(op)]

            if not non_tabu:
                break

            best_op = non_tabu[0]
            best_delta = best_op.get_delta_cost(current)

            for op in non_tabu[1:]:
                delta = op.get_delta_cost(current)
                # Comme delta = (Nouvelle - Ancienne), un delta négatif est une baisse de distance.
                # On cherche donc à MINIMISER le delta.
                if delta < best_delta:
                    best_op = op
                    best_delta = delta

            best_solution = best_op.apply(current)

            # Si le delta >= 0, la distance augmente ou stagne (mauvais mouvement), on l'ajoute en Tabou.
            if best_delta >= 0:
                self._add_tabu(best_op)

            current = best_solution
            yield current   