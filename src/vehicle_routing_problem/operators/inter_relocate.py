from typing import override
from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.route import Route
from vehicle_routing_problem.operators.base_operator import BaseOperator


class IntraRelocate(BaseOperator):

    def __init__(self, instance, route1_id: int, route2_id: int, client: int, insert_pos: int):
        super().__init__(instance)
        self.route1_id = route1_id
        self.route2_id = route2_id
        self.client = client
        self.insert_pos = insert_pos

    @override
    def apply(self, solution: Solution) -> Solution:
        new_solution = solution.copy()

        # vérification indices routes
        if self.route1_id >= len(new_solution.routes) or self.route2_id >= len(new_solution.routes):
            return new_solution

        route1 = new_solution.routes[self.route1_id]
        route2 = new_solution.routes[self.route2_id]

        # vérification client
        if self.client >= len(route1.client_ids):
            return new_solution

        # retirer client de route1
        client_id = route1.client_ids.pop(self.client)

        # position d'insertion valide
        insert_pos = min(self.insert_pos, len(route2.client_ids))

        # insérer dans route2
        route2.client_ids.insert(insert_pos, client_id)

        return new_solution


    @classmethod
    @override
    def generate_neighbors(cls, instance: Instance, solution: Solution) -> list[BaseOperator]:
        neighbors = []

        for r1, route1 in enumerate(solution.routes):
            for r2, route2 in enumerate(solution.routes):
                for i in range(len(route1.client_ids)):
                    for j in range(len(route2.client_ids) + 1):
                        neighbors.append(
                            cls(instance, r1, r2, i, j)
                        )

        return neighbors