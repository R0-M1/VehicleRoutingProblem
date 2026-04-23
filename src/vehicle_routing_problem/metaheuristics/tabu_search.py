from __future__ import annotations
from collections import deque
import random
from typing import Iterator

from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.metaheuristics.base_metaheuristic import BaseMetaheuristic
from vehicle_routing_problem.operators.base_operator import BaseOperator


class TabuSearch(BaseMetaheuristic):

    def __init__(self, instance: Instance, tabu_size: int, n_neighbors: int = 100, check_time_windows: bool = True):
        super().__init__(instance)
        self._tabu_size = tabu_size
        self._n_neighbors = n_neighbors
        self._check_time_windows = check_time_windows
        self.tabu_list: deque[tuple] = deque(maxlen=self._tabu_size)
        self.tabu_set: set[tuple] = set()

    def _is_tabu(self, op: BaseOperator) -> bool:
        return op.get_signature() in self.tabu_set

    def _add_tabu(self, op: BaseOperator):
        signature = op.get_signature()
        if len(self.tabu_list) == self._tabu_size:
            old_signature = self.tabu_list.popleft()
            self.tabu_set.discard(old_signature)
        self.tabu_list.append(signature)
        self.tabu_set.add(signature)

    def _is_feasible(self, solution: Solution, affected: list[int]) -> bool:
        for i in affected:
            route = solution.routes[i]
            if not route.is_capacity_feasible:
                return False
            if self._check_time_windows and not route.is_time_feasible():
                return False
        return True

    def solve(self, current_solution: Solution) -> Iterator[Solution]:
        self.tabu_list.clear()
        self.tabu_set.clear()
        current = current_solution.copy()

        while True:
            all_operators = BaseOperator.get_operators(self._inst)

            sampled_neighbors = []
            available_ops = list(all_operators)
            attempts = 0
            max_attempts = self._n_neighbors * 10

            while len(sampled_neighbors) < self._n_neighbors and available_ops and attempts < max_attempts:
                attempts += 1
                op_type = random.choice(available_ops)
                neighbor = op_type.sample_random_neighbor(self._inst, current)

                if neighbor is not None:
                    sampled_neighbors.append(neighbor)
                else:
                    available_ops.remove(op_type)

            if not sampled_neighbors:
                break

            non_tabu = [op for op in sampled_neighbors if not self._is_tabu(op)]

            if not non_tabu:
                break

            best_op = None
            best_delta = float('inf')

            for op in non_tabu:
                delta = op.get_delta_cost(current)
                if delta >= best_delta:
                    continue
                # On ne calcule apply() que si le delta est prometteur
                candidate = op.apply(current)
                if not self._is_feasible(candidate, op.affected_routes()):
                    continue
                best_op = op
                best_delta = delta

            if best_op is None:
                break

            current = best_op.apply(current)

            if best_delta >= 0:
                self._add_tabu(best_op)

            yield current