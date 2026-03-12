from typing import override

from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.operators.base_operator import BaseOperator


class IntraExchange(BaseOperator):

    def __init__(self, instance, route1, client1, client2):
        super().__init__(instance)
        self.route1 = route1
        self.client1 = client1
        self.client2 = client2

    @override
    def apply(self, solution: Solution) -> Solution:
        new_solution = solution.copy()

        if self.route1 >= len(new_solution.routes):
            return new_solution

        route = new_solution.routes[self.route1]

        if self.client1 >= len(route.client_ids) or self.client2 >= len(route.client_ids):
            return new_solution

        # échange des deux clients
        route.client_ids[self.client1], route.client_ids[self.client2] = (
            route.client_ids[self.client2],
            route.client_ids[self.client1],
        )

        return new_solution
    
    @override
    def generate_neighbors(self, solution: Solution) -> list[BaseOperator]:
        """
        Génère tous les échanges possibles intra-route pour chaque route.
        """
        neighbors = []
        for route_idx, route in enumerate(solution.routes):
            n = len(route.client_ids)
            # tous les couples (i,j) avec i < j
            for i in range(n):
                for j in range(i + 1, n):
                    # créer un opérateur qui échange i et j dans route_idx
                    op = IntraExchange(self._inst, route_idx, i, j)
                    neighbors.append(op)
        return neighbors
