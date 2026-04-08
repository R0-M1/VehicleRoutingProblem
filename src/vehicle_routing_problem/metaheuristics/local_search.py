from __future__ import annotations

from random import sample
from typing import Iterator

from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.metaheuristics.base_metaheuristic import BaseMetaheuristic
from vehicle_routing_problem.operators.base_operator import BaseOperator


class LocalSearch(BaseMetaheuristic):
    """
    Local Search (Descente de gradient) : à chaque itération, applique le
    meilleur mouvement d'amélioration trouvé dans TOUS les voisinages.
    Arrête quand plus d'amélioration possible.
    """

    def __init__(self, instance: Instance, n_neighbors: int = 100):
        super().__init__(instance)
        self._n_neighbors = n_neighbors

    def solve(self, current_solution: Solution) -> Iterator[Solution]:
        current = current_solution.copy()

        while True:
            all_operators = BaseOperator.get_operators(self._inst)

            all_neighbors: list[BaseOperator] = []
            for op_type in all_operators:
                all_neighbors.extend(op_type.generate_neighbors(self._inst, current))

            if not all_neighbors:
                break

            sampled_neighbors = sample(all_neighbors, k=min(100, len(all_neighbors)))

            best_op = None
            best_improvement = 0.0

            for op in sampled_neighbors:
                neighbor = op.apply(current)
                improvement = current.total_distance - neighbor.total_distance

                if improvement > best_improvement:
                    best_improvement = improvement
                    best_op = op

            if best_op is None or best_improvement <= 0:
                break

            current = best_op.apply(current)
            yield current
