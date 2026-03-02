from dataclasses import dataclass
from typing import List
import numpy as np

@dataclass
class Node:
    id: int
    x: float
    y: float
    demand: int
    tw_start: int
    tw_end: int
    service_time: int


class Route:
    def __init__(self, nodes: List[Node] = None):
        # Une route commence et finit toujours par le dépôt (Node 0)
        self.nodes = nodes if nodes else []

    def is_valid(self, max_capacity: int, dist_matrix: np.ndarray, use_tw: bool = True) -> bool:
        """Vérifie la capacité C et les fenêtres de temps (si use_tw=True)"""
        current_time = 0.0
        current_load = 0

        for i in range(len(self.nodes) - 1):
            current_node = self.nodes[i]
            next_node = self.nodes[i + 1]

            # 1. Vérification de la capacité
            current_load += current_node.demand
            if current_load > max_capacity:
                return False

            # 2. Avancée du temps (trajet + service)
            if i > 0:  # Pas de temps de service au dépôt de départ
                current_time += current_node.service_time

            current_time += dist_matrix[current_node.id][next_node.id]

            # 3. Vérification des fenêtres de temps
            if use_tw:
                if current_time > next_node.tw_end:
                    return False  # Arrivé trop tard !
                if current_time < next_node.tw_start:
                    current_time = next_node.tw_start  # Attente autorisée si arrivé en avance

        return True

    def distance(self, dist_matrix: np.ndarray) -> float:
        """Calcule la distance totale de cette tournée"""
        return sum(dist_matrix[self.nodes[i].id][self.nodes[i + 1].id]
                   for i in range(len(self.nodes) - 1))


class Solution:
    def __init__(self, routes: List[Route]):
        self.routes = routes

    def total_distance(self, dist_matrix: np.ndarray) -> float:
        return sum(route.distance(dist_matrix) for route in self.routes)

    def total_vehicles(self) -> int:
        return len(self.routes)
