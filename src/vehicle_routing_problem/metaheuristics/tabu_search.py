from __future__ import annotations
from collections import deque
from random import sample
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
        self.tabu_list: deque[BaseOperator] = deque(maxlen=self._tabu_size)

    @staticmethod
    def _is_tabu(op: BaseOperator, tabu_list: deque) -> bool:
        """Comparaison sémantique des opérateurs par leurs attributs."""
        return any(op.__dict__ == tabu_op.__dict__ for tabu_op in tabu_list)

    def solve(self, current_solution: Solution) -> Iterator[Solution]:
        self.tabu_list.clear()
        current = current_solution.copy()

        while True:
            all_operators = BaseOperator.get_operators(self._inst)

            all_neighbors: list[BaseOperator] = []
            for op_type in all_operators:
                all_neighbors.extend(op_type.generate_neighbors(self._inst, current))

            sampled_neighbors = sample(all_neighbors, k=min(100, len(all_neighbors)))

            if not all_neighbors:
                break

            non_tabu = [op for op in sampled_neighbors if not self._is_tabu(op, self.tabu_list)]

            if not non_tabu:
                break

            best_op = non_tabu[0]
            best_solution = best_op.apply(current)

            for op in non_tabu[1:]:
                candidate = op.apply(current)
                if candidate.total_distance < best_solution.total_distance:
                    best_op = op
                    best_solution = candidate

            if best_solution.total_distance >= current.total_distance:
                self.tabu_list.append(best_op)

            current = best_solution
            yield current