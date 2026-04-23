from typing import List, override

from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.operators.base_operator import BaseOperator


class InterExchange(BaseOperator):
    """
    Opérateur IntraExchange inter-route : échange un client de la route1
    avec un client de la route2.
    """

    def __init__(self, instance, route1_id: int, route2_id: int, client1: int, client2: int):
        super().__init__(instance)
        self.route1_id = route1_id
        self.route2_id = route2_id
        self.client1 = client1
        self.client2 = client2

    @override
    def apply(self, solution: Solution) -> Solution:
        new_solution = solution.copy()

        # Vérification indices de routes
        if self.route1_id >= len(new_solution.routes) or self.route2_id >= len(new_solution.routes):
            return new_solution

        route1 = new_solution.routes[self.route1_id]
        route2 = new_solution.routes[self.route2_id]

        # Vérification indices des clients
        if self.client1 >= len(route1.client_ids) or self.client2 >= len(route2.client_ids):
            return new_solution

        # Échange des clients
        route1.client_ids[self.client1], route2.client_ids[self.client2] = (
            route2.client_ids[self.client2],
            route1.client_ids[self.client1],
        )

        return new_solution

    def get_delta_cost(self, solution: Solution) -> float:
        """
        Calcule la différence de coût pour un échange de clients entre deux routes en $O(1)$.
        On casse 4 arêtes et on en recrée 4.
        """
        if self.route1_id >= len(solution.routes) or self.route2_id >= len(solution.routes):
            return 0.0

        route1 = solution.routes[self.route1_id]
        route2 = solution.routes[self.route2_id]

        if self.client1 >= len(route1.client_ids) or self.client2 >= len(route2.client_ids):
            return 0.0

        # On rajoute les dépôts (id=0) virtuellement
        ids1 = [0] + route1.client_ids + [0]
        ids2 = [0] + route2.client_ids + [0]
        dm = self._inst.dist_matrix

        # Indices ajustés (+1 pour les dépôts)
        u1 = self.client1 + 1
        u2 = self.client2 + 1

        # Nœuds impliqués dans la route 1
        c1 = ids1[u1]      # Le client échangé de la route 1
        p1 = ids1[u1 - 1]  # Précédent
        n1 = ids1[u1 + 1]  # Suivant

        # Nœuds impliqués dans la route 2
        c2 = ids2[u2]      # Le client échangé de la route 2
        p2 = ids2[u2 - 1]  # Précédent
        n2 = ids2[u2 + 1]  # Suivant

        # On casse 2 arêtes dans la route 1 et 2 dans la route 2
        removed = (
            dm[p1][c1] + dm[c1][n1] +
            dm[p2][c2] + dm[c2][n2]
        )

        # On insère c2 à la place de c1, et c1 à la place de c2
        added = (
            dm[p1][c2] + dm[c2][n1] +
            dm[p2][c1] + dm[c1][n2]
        )

        return float(added - removed)

    @classmethod
    @override
    def generate_neighbors(cls, instance: Instance, solution: Solution) -> List[BaseOperator]:
        """
        Génère tous les échanges possibles entre toutes les paires de routes
        pour tous les clients dans chaque route.
        """
        neighbors = []

        n_routes = len(solution.routes)
        for r1 in range(n_routes):
            for r2 in range(r1 + 1, n_routes):
                route1 = solution.routes[r1]
                route2 = solution.routes[r2]

                for i in range(len(route1.client_ids)):
                    for j in range(len(route2.client_ids)):
                        neighbors.append(cls(instance, r1, r2, i, j))

        return neighbors