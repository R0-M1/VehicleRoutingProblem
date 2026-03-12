from typing import override

from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.operators.base_operator import BaseOperator


class IntraReverse(BaseOperator):

    def __init__(self, instance, route_id):
        super().__init__(instance)
        self.route_id = route_id

    @override
    def apply(self, solution: Solution) -> Solution:
        new_solution = solution.copy()

        route = new_solution.routes[self.route_id]
        route.client_ids.reverse()

        return new_solution

    @override
    def generate_neighbors(self, solution: Solution) -> list[BaseOperator]:
        """
        Génère tous les échanges possibles intra-route pour chaque route.
        """
        neighbors = []
        for route_id, route in enumerate(solution.routes):
            for i in range(route.cl):
                neighbors.append(IntraReverse(self._inst, route_id))
        return neighbors
