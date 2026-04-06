from typing import override
from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.route import Route
from vehicle_routing_problem.operators.base_operator import BaseOperator


class InterCrossExchange(BaseOperator):

    def __init__(
        self,
        instance: Instance,
        route1_id: int,
        route2_id: int,
        start1: int,
        end1: int,
        start2: int,
        end2: int,
    ):
        super().__init__(instance)
        self.route1_id = route1_id
        self.route2_id = route2_id
        self.start1 = start1
        self.end1 = end1
        self.start2 = start2
        self.end2 = end2

    @override
    def apply(self, solution: Solution) -> Solution:

        new_solution = solution.copy()

        if (
            self.route1_id >= len(new_solution.routes)
            or self.route2_id >= len(new_solution.routes)
        ):
            return new_solution

        route1 = new_solution.routes[self.route1_id]
        route2 = new_solution.routes[self.route2_id]

        ids1 = route1.client_ids
        ids2 = route2.client_ids

        if (
            self.end1 >= len(ids1)
            or self.end2 >= len(ids2)
            or self.start1 > self.end1
            or self.start2 > self.end2
        ):
            return new_solution

        # segments à échanger
        seg1 = ids1[self.start1 : self.end1 + 1]
        seg2 = ids2[self.start2 : self.end2 + 1]

        # nouvelles routes
        new_ids1 = ids1[: self.start1] + seg2 + ids1[self.end1 + 1 :]
        new_ids2 = ids2[: self.start2] + seg1 + ids2[self.end2 + 1 :]

        new_route1 = Route(new_ids1, self._inst)
        new_route2 = Route(new_ids2, self._inst)

        # # vérification capacité
        # if not new_route1.is_capacity_feasible or not new_route2.is_capacity_feasible:
        #     return new_solution

        new_solution.routes[self.route1_id] = new_route1
        new_solution.routes[self.route2_id] = new_route2

        return new_solution

    @classmethod
    @override
    def generate_neighbors(
        cls, instance: Instance, solution: Solution
    ) -> list[BaseOperator]:

        neighbors = []
        routes = solution.routes
        n_routes = len(routes)

        for r1 in range(n_routes):
            for r2 in range(r1 + 1, n_routes):
                route1 = routes[r1]
                route2 = routes[r2]
                n1 = len(route1.client_ids)
                n2 = len(route2.client_ids)
                for i in range(n1):
                    for j in range(i, n1):
                        for k in range(n2):
                            for l in range(k, n2):

                                neighbors.append(
                                    cls(instance, r1, r2, i, j, k, l)
                                )
        return neighbors