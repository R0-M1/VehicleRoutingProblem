from typing import List, override

from vehicle_routing_problem.core.instance import Instance
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

    def get_delta_cost(self, solution: Solution) -> float:
        """
        Calcule la différence de coût pour un 2-opt.
        En supposant une matrice de distance symétrique (Euclidienne), les arêtes
        à l'intérieur du segment inversé gardent la même longueur. 
        On ne calcule donc que l'impact des arêtes cassées et recréées aux extrémités.
        """
        if self.route_id >= len(solution.routes):
            return 0.0

        route = solution.routes[self.route_id]
        
        n = len(route.client_ids)
        if self.client1 >= n or self.client2 >= n or self.client1 >= self.client2:
            return 0.0

        # Ajout des dépôts
        ids = [0] + route.client_ids + [0]
        dm = self._inst.dist_matrix

        # Indices ajustés (+1 car le dépôt est à l'index 0 de "ids")
        i = self.client1 + 1
        j = self.client2 + 1

        ci = ids[i]         # Premier noeud du segment à inverser
        cj = ids[j]         # Dernier noeud du segment à inverser
        pi = ids[i - 1]     # Le noeud AVANT le segment
        nj = ids[j + 1]     # Le noeud APRÈS le segment

        # Pour le 2-opt, on casse deux arêtes (avant et après le segment):
        # pi -> ci   et   cj -> nj
        removed = dm[pi][ci] + dm[cj][nj]
        
        # Et on recrée deux arêtes (en croisant, puisque le segment est inversé):
        # pi -> cj   et   ci -> nj
        added = dm[pi][cj] + dm[ci][nj]

        # Note: si les distances étaient asymétriques (ex: routes avec du vent), 
        # il faudrait aussi recalculer tout l'intérieur du segment qui est inversé.
        # Ici la matrice de l'instance est Euclidienne (symétrique), donc c'est O(1).

        return float(added - removed)

    @classmethod
    @override
    def generate_neighbors(cls, instance: Instance, solution: Solution) -> List[BaseOperator]:
        """
        Génère tous les voisins possibles intra-route pour chaque route
        en inversant toutes les paires (i,j) avec i < j.
        """
        neighbors = []

        for route_id, route in enumerate(solution.routes):
            n = len(route.client_ids)
            # toutes les paires i < j
            for i in range(n):
                for j in range(i + 1, n):
                    op = cls(instance, route_id, i, j)
                    neighbors.append(op)

        return neighbors
