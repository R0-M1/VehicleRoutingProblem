from typing import override
from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.route import Route
from vehicle_routing_problem.operators.base_operator import BaseOperator


class InterRelocate(BaseOperator):

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

    def get_delta_cost(self, solution: Solution) -> float:
        """
        Calcule la différence de coût pour un déplacement de client d'une route à une ESPÈCE AUTRE.
        O(1) en cassant 3 arêtes et recréant 3 arêtes.
        """
        if self.route1_id >= len(solution.routes) or self.route2_id >= len(solution.routes):
            return 0.0
            
        # Par sécurité, si l'opérateur est appelé sur la même route, le delta serait l'équivalent IntraRelocate
        if self.route1_id == self.route2_id:
            # Soit on le recalcule comme IntraRelocate, soit on l'ignore car ce doublon 
            # de mouvement est supprimé dans generate_neighbors ci-dessous.
            return 0.0

        route1 = solution.routes[self.route1_id]
        route2 = solution.routes[self.route2_id]

        if self.client >= len(route1.client_ids):
            return 0.0

        dm = self._inst.dist_matrix

        ids1 = [0] + route1.client_ids + [0]
        ids2 = [0] + route2.client_ids + [0]

        # ---- ROUTE 1 (Extraction) ----
        u = self.client + 1
        c = ids1[u]
        
        p1 = ids1[u - 1]
        n1 = ids1[u + 1]

        # ---- ROUTE 2 (Insertion) ----
        insert_pos = min(self.insert_pos, len(route2.client_ids))
        p2 = ids2[insert_pos]      # Le noeud juste avant l'endroit d'insertion
        n2 = ids2[insert_pos + 1]  # Le noeud juste après l'endroit d'insertion

        # On casse 3 arêtes :
        # - Les 2 arêtes de 'c' dans la route 1
        # - L'arête à l'endroit d'insertion dans la route 2
        removed = (
            dm[p1][c] + dm[c][n1] +
            dm[p2][n2]
        )

        # On recrée 3 arêtes :
        # - On bouche le trou dans la route 1 (les anciens voisins de c se relient)
        # - On insère 'c' dans la route 2 (entre p2 et n2)
        added = (
            dm[p1][n1] +
            dm[p2][c] + dm[c][n2]
        )

        return float(added - removed)

    @classmethod
    @override
    def generate_neighbors(cls, instance: Instance, solution: Solution) -> list[BaseOperator]:
        neighbors = []

        for r1, route1 in enumerate(solution.routes):
            for r2, route2 in enumerate(solution.routes):
                # OPTIMISATION MAJEURE : On évite de générer les doublons de IntraRelocate
                if r1 == r2:
                    continue

                for i in range(len(route1.client_ids)):
                    for j in range(len(route2.client_ids) + 1):
                        neighbors.append(
                            cls(instance, r1, r2, i, j)
                        )

        return neighbors

    @classmethod
    @override
    def sample_random_neighbor(cls, instance: Instance, solution: Solution) -> BaseOperator | None:
        import random
        if len(solution.routes) < 2:
            return None
        valid_r1 = [r_id for r_id, r in enumerate(solution.routes) if len(r.client_ids) > 0]
        if not valid_r1:
            return None
        r1 = random.choice(valid_r1)
        r2 = random.randrange(len(solution.routes))
        while r1 == r2:
            r2 = random.randrange(len(solution.routes))
            
        i = random.randrange(len(solution.routes[r1].client_ids))
        j = random.randrange(len(solution.routes[r2].client_ids) + 1)
        return cls(instance, r1, r2, i, j)