import random
from typing import List
import numpy as np
from vehicle_routing_problem.core import Node, Route, Solution


def generate_random_solution(nodes: List[Node], capacity: int, dist_matrix: np.ndarray,
                             use_tw: bool = True) -> Solution:
    """Génère une solution aléatoire valide en créant de nouveaux véhicules si besoin"""
    depot = nodes[0]
    customers = nodes[1:]

    # Mélange aléatoire des clients
    random.shuffle(customers)

    routes = []
    current_route_nodes = [depot]

    for customer in customers:
        # On tente d'ajouter le client à la route actuelle
        test_route = Route(current_route_nodes + [customer, depot])

        # Si ça casse la capacité ou le temps, on ferme la route et on crée un nouveau camion
        if not test_route.is_valid(capacity, dist_matrix, use_tw):
            # On sauvegarde la route précédente (sans le nouveau client)
            routes.append(Route(current_route_nodes + [depot]))
            # On met le client dans un nouveau camion
            current_route_nodes = [depot, customer]
        else:
            current_route_nodes.append(customer)

    # Ajouter la dernière route
    if len(current_route_nodes) > 1:
        routes.append(Route(current_route_nodes + [depot]))

    return Solution(routes)
