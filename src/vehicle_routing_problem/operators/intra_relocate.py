from typing import override

from vehicle_routing_problem.core.instance import Instance
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

    def get_delta_cost(self, solution: Solution) -> float:
        """
        Calcule la différence de coût pour un déplacement de client (Relocate) en $O(1)$.
        """
        if self.route_id >= len(solution.routes):
            return 0.0

        route = solution.routes[self.route_id]
        n_clients = len(route.client_ids)

        if self.client1 >= n_clients or self.client2 >= n_clients or self.client1 == self.client2:
            return 0.0

        ids = [0] + route.client_ids + [0]
        dm = self._inst.dist_matrix

        u = self.client1 + 1  # L'index du client déplacé dans `ids`

        # Détermination des points d'insertion (qui sera avant et après le client déplacé ?)
        if self.client1 < self.client2:
            ins_prev = self.client2 + 1
            ins_next = self.client2 + 2
        else: # self.client1 > self.client2
            ins_prev = self.client2
            ins_next = self.client2 + 1

        # 3 arêtes sont brisées :
        # L'ancienne arête avant le client déplacé, l'arête après, et l'arête à l'endroit de l'insertion.
        removed = (
            dm[ids[u-1]][ids[u]] + dm[ids[u]][ids[u+1]] 
            + dm[ids[ins_prev]][ids[ins_next]]
        )

        # 3 arêtes sont recréées :
        # On relie les anciens voisins du client déplacé, et on insère le client avec ses nouveaux voisins.
        added = (
            dm[ids[u-1]][ids[u+1]] 
            + dm[ids[ins_prev]][ids[u]] + dm[ids[u]][ids[ins_next]]
        )

        return float(added - removed)

    @classmethod
    @override
    def generate_neighbors(cls, instance: Instance, solution: Solution) -> list[BaseOperator]:
        """
        Génère tous les déplacements (relocate) possibles au sein de chaque route.
        """
        neighbors = []

        for route_id, route in enumerate(solution.routes):
            n = len(route.client_ids)
            for i in range(n):
                # tous les couples (i, j) avec i < j
                for j in range(i+1,n):
                    op = cls(instance, route_id, i, j)
                    neighbors.append(op)

        return neighbors

    @classmethod
    @override
    def sample_random_neighbor(cls, instance: Instance, solution: Solution) -> BaseOperator | None:
        import random
        valid_routes = [r_id for r_id, r in enumerate(solution.routes) if len(r.client_ids) >= 2]
        if not valid_routes:
            return None
        route_id = random.choice(valid_routes)
        n = len(solution.routes[route_id].client_ids)
        i, j = random.sample(range(n), 2)
        if i > j:
            i, j = j, i
        return cls(instance, route_id, i, j)