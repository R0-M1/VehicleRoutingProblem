from typing import override

from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.operators.base_operator import BaseOperator


class IntraReverse(BaseOperator):

    def __init__(self, instance: Instance, route_id: int):
        super().__init__(instance)
        self.route_id = route_id

    @override
    def apply(self, solution: Solution) -> Solution:
        new_solution = solution.copy()

        if self.route_id >= len(new_solution.routes):
            return new_solution

        route = new_solution.routes[self.route_id]
        route.client_ids.reverse()

        return new_solution

    @override
    def generate_neighbors(self, solution: Solution) -> list[BaseOperator]:
        """
        Génère tous les échanges possibles intra-route pour chaque route.
        """
        return [IntraReverse(self._inst, route_id)
            for route_id, route in enumerate(solution.routes)]
