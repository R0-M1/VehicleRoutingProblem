from __future__ import annotations

class Route:
    """
    Une tournée = liste ordonnée d'IDs (entiers).
    Le dépôt (id=0) est implicite au début et à la fin.
    """
    def __init__(self, client_ids: list[int], instance):
        self.client_ids = client_ids
        self._inst = instance

    @property
    def distance(self) -> float:
        ids = [0] + self.client_ids + [0]
        dm  = self._inst.dist_matrix
        return sum(dm[ids[i]][ids[i+1]] for i in range(len(ids) - 1))

    @property
    def total_demand(self) -> int:
        return sum(self._inst.clients[i].demand for i in self.client_ids)

    @property
    def is_capacity_feasible(self) -> bool:
        return self.total_demand <= self._inst.capacity

    def copy(self) -> Route:
        return Route(self.client_ids.copy(), self._inst)

    def __len__(self):
        return len(self.client_ids)

    def __repr__(self):
        seq = " → ".join(map(str, self.client_ids))
        return f"Route(0 → {seq} → 0  |  d={self.distance:.1f}  |  load={self.total_demand})"
