from __future__ import annotations
from .route import Route

class Solution:
    def __init__(self, routes: list[Route]):
        self.routes = [r for r in routes if len(r) > 0]  # ignore routes vides

    @property
    def total_distance(self) -> float:
        return sum(r.distance for r in self.routes)

    @property
    def nb_vehicles(self) -> int:
        return len(self.routes)

    def all_clients_visited(self, instance) -> bool:
        visited = [cid for r in self.routes for cid in r.client_ids]
        expected = list(range(1, len(instance.clients)))
        return sorted(visited) == sorted(expected)

    def copy(self) -> Solution:
        return Solution([r.copy() for r in self.routes])

    def __repr__(self):
        return (f"Solution | véhicules={self.nb_vehicles} "
                f"| distance totale={self.total_distance:.2f}")
