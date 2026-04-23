from typing import override

from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.operators.base_operator import BaseOperator


class IntraReverse(BaseOperator):

    def __init__(self, instance: Instance, route_id: int):
        super().__init__(instance)
        self.route_id = route_id

    @override
    def apply(self, solution: Solution) -> Solution:
        new_solution = solution.copy()

        if self.route_id >= len(new_solution.routes):
            return new_solution

        route = new_solution.routes[self.route_id]
        route.client_ids.reverse()

        return new_solution

    def get_delta_cost(self, solution: Solution) -> float:
        """
        Calcule la différence de coût.
        Dans le cas de distances euclidiennes (symétriques), inverser la totalité 
        de la route ne change absolument pas sa distance totale. (Ex: Le chemin
        Dépôt -> A -> B -> Dépôt a la même longueur que Dépôt -> B -> A -> Dépôt).
        Le coût (Delta) est donc toujours exactement 0 !
        """
        if self.route_id >= len(solution.routes):
            return 0.0

        # Si un jour tu utilises des distances asymétriques (ex: routes en sens unique),
        # il faudrait boucler sur toute la route ici (O(n)) pour recalculer les distances à l'envers.
        # Mais pour des points 2D géométriques, c'est instantané.
        return 0.0

    @classmethod
    @override
    def generate_neighbors(cls, instance: Instance, solution: Solution) -> list[BaseOperator]:
        """
        Génère tous les échanges possibles intra-route pour chaque route.
        """
        return [cls(instance, route_id)
                for route_id, route in enumerate(solution.routes)]

    @classmethod
    @override
    def sample_random_neighbor(cls, instance: Instance, solution: Solution) -> BaseOperator | None:
        if not solution.routes:
            return None
        import random
        route_id = random.randrange(len(solution.routes))
        return cls(instance, route_id)
