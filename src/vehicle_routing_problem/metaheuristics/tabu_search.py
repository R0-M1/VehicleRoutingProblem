from __future__ import annotations
from collections import deque
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

    def __init__(self, instance: Instance, tabu_size: int):
        super().__init__(instance)
        self._tabu_size = tabu_size

    @staticmethod
    def _is_tabu(op: BaseOperator, tabu_list: deque) -> bool:
        """Comparaison sémantique des opérateurs par leurs attributs."""
        return any(op.__dict__ == tabu_op.__dict__ for tabu_op in tabu_list)

    def solve(self, current_solution: Solution) -> Iterator[Solution]:
        tabu_list: deque[BaseOperator] = deque(maxlen=self._tabu_size)
        current = current_solution.copy()

        while True:
            all_operators = BaseOperator.get_operators(self._inst)

            all_neighbors: list[BaseOperator] = []
            for op_type in all_operators:
                all_neighbors.extend(op_type.generate_neighbors(self._inst, current))

            non_tabu = [op for op in all_neighbors if not self._is_tabu(op, tabu_list)]

            if not non_tabu:
                break

            best_op = non_tabu[0]
            for op in non_tabu[1:]:
                if op.apply(current).total_distance < best_op.apply(current).total_distance:
                    best_op = op

            current = best_op.apply(current)
            tabu_list.append(best_op)

            yield current
