import copy
from vehicle_routing_problem.model import Route, Solution
import numpy as np


def intra_relocate(route: Route, capacity: int, dist_matrix: np.ndarray, use_tw: bool) -> Route:
    """Prend un client et l'insère ailleurs dans la MÊME route (si valide)"""
    best_route = route
    min_dist = route.distance(dist_matrix)

    n = len(route.nodes)
    if n <= 3:  # Juste Dépôt -> 1 client -> Dépôt, impossible de permuter
        return route

    # On teste tous les déplacements possibles
    for i in range(1, n - 1):
        for j in range(1, n - 1):
            if i == j: continue

            # Créer une copie de la route
            new_nodes = route.nodes.copy()
            # Retirer le client i
            customer = new_nodes.pop(i)
            # L'insérer à la position j
            new_nodes.insert(j, customer)

            test_route = Route(new_nodes)

            # Si le mouvement respecte les contraintes et améliore la distance
            if test_route.is_valid(capacity, dist_matrix, use_tw):
                dist = test_route.distance(dist_matrix)
                if dist < min_dist:
                    min_dist = dist
                    best_route = test_route

    return best_route
