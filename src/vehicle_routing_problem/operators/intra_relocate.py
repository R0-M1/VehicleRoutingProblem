from typing import override
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.route import Route
from vehicle_routing_problem.operators.base_operator import BaseOperator

class IntraRelocate(BaseOperator):
    """
    Relocate intra-route : retire le client en position i et le réinsère 
    en position j au sein de la même route[cite: 97].
    """

    def __init__(self, instance, route_idx: int, i: int, j: int):
        super().__init__(instance)
        self.route_idx = route_idx
        self.i = i  
        self.j = j  

    @override
    def apply(self, solution: Solution) -> Solution:
        new_solution = solution.copy()

        if self.route_idx >= len(new_solution.routes):
            return new_solution

        old_route = new_solution.routes[self.route_idx]
        new_ids = old_route.client_ids.copy()

        if self.i >= len(new_ids) or self.j >= len(new_ids):
            return new_solution

        client = new_ids.pop(self.i)
        new_ids.insert(self.j, client)


        new_solution.routes[self.route_idx] = Route(new_ids, self._inst)
        
        return new_solution


    @override
    def generate_neighbors(self, solution: Solution) -> list[BaseOperator]:
        """
        Génère tous les déplacements (relocate) possibles au sein de chaque route.
        """
        neighbors = []
        for route_idx, route in enumerate(solution.routes):
            n = len(route.client_ids)
            for i in range(n):
                for j in range(n):
                    if i == j:
                        continue
                    op = IntraRelocate(self._inst, route_idx, i, j)
                    neighbors.append(op)
        return neighbors