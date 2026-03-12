from typing import Iterator, List, override

from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.operators.base_operator import BaseOperator


class IntraExchange(BaseOperator):

    def __init__(self, instance, route_id: int, client1: int, client2: int):
        super().__init__(instance)
        self.route_id = route_id
        self.client1 = client1
        self.client2 = client2

    @override
    def apply(self, solution: Solution) -> Solution:
        new_solution = solution.copy()

        # Vérification de l'indice de route
        if self.route_id >= len(new_solution.routes):
            return new_solution

        route = new_solution.routes[self.route_id]

        # Vérification des indices de clients
        if self.client1 >= len(route.client_ids) or self.client2 >= len(route.client_ids):
            return new_solution

        # Échange des deux clients
        route.client_ids[self.client1], route.client_ids[self.client2] = (
            route.client_ids[self.client2],
            route.client_ids[self.client1],
        )

        return new_solution

    @classmethod
    @override
    def generate_neighbors(cls, instance: Instance, solution: Solution) -> List[BaseOperator]:
        """
        Génère tous les échanges possibles intra-route pour chaque route.
        """
        neighbors = []

        for route_id, route in enumerate(solution.routes):
            n = len(route.client_ids)
            # tous les couples (i, j) avec i < j
            for i in range(n):
                for j in range(i + 1, n):
                    op = cls(instance, route_id, i, j)
                    neighbors.append(op)

        return neighbors