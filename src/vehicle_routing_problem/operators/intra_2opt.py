from typing import List, override
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.operators.base_operator import BaseOperator


class Intra2Opt(BaseOperator):
    """
    Opérateur Intra2Opt : inverse une sous-liste de clients dans une route.
    """

    def __init__(self, instance, route_id: int, client1: int, client2: int):
        super().__init__(instance)
        self.route_id = route_id
        self.client1 = client1
        self.client2 = client2

    @override
    def apply(self, solution: Solution) -> Solution:
        new_solution = solution.copy()

        # vérification de l'indice de route
        if self.route_id >= len(new_solution.routes):
            return new_solution

        route = new_solution.routes[self.route_id]

        # vérification des indices
        n = len(route.client_ids)
        if self.client1 >= n or self.client2 >= n or self.client1 >= self.client2:
            return new_solution

        # inversion de la sous-liste
        route.client_ids[self.client1 : self.client2 + 1] = reversed(route.client_ids[self.client1 : self.client2 + 1])

        return new_solution

    @override
    def generate_neighbors(self, solution: Solution) -> List[BaseOperator]:
        """
        Génère tous les voisins possibles intra-route pour chaque route
        en inversant toutes les paires (i,j) avec i < j.
        """
        neighbors: List[BaseOperator] = []

        for route_idx, route in enumerate(solution.routes):
            n = len(route.client_ids)
            # toutes les paires i < j
            for i in range(n):
                for j in range(i + 1, n):
                    op = Intra2Opt(self._inst, route_idx, i, j)
                    neighbors.append(op)

        return neighbors
