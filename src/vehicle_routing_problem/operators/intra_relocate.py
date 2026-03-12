from typing import override
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.route import Route
from vehicle_routing_problem.operators.base_operator import BaseOperator

class IntraRelocate(BaseOperator):
    """
    Relocate intra-route : retire le client en position i et le réinsère 
    en position j au sein de la même route[cite: 97].
    """

    def __init__(self, instance, route_id: int, client1: int, client2: int):
        super().__init__(instance)
        self.route_id = route_id
        self.client1 = client1
        self.client2 = client2 

    @override
    def apply(self, solution: Solution) -> Solution:
        new_solution = solution.copy()

        if self.route_id >= len(new_solution.routes):
            return new_solution

        old_route = new_solution.routes[self.route_id]
        new_ids = old_route.client_ids.copy()

        if self.client1 >= len(new_ids) or self.client2 >= len(new_ids):
            return new_solution

        client = new_ids.pop(self.client1)
        new_ids.insert(self.client2, client)


        new_solution.routes[self.route_id] = Route(new_ids, self._inst)
        
        return new_solution


    @override
    def generate_neighbors(self, solution: Solution) -> list[BaseOperator]:
        """
        Génère tous les déplacements (relocate) possibles au sein de chaque route.
        """
        neighbors = []
        for route_id, route in enumerate(solution.routes):
            n = len(route.client_ids)
            for i in range(n):
                for j in range(n):
                    if i == j:
                        continue
                    op = IntraRelocate(self._inst, route_id, i, j)
                    neighbors.append(op)
        return neighbors