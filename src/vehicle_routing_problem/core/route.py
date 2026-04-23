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

    def is_time_feasible(self) -> bool:
        time = 0
        inst = self._inst
        dm = inst.dist_matrix
        prev = 0

        for cid in self.client_ids:
            client = inst.clients[cid]

            travel = dm[prev][cid]
            time += travel

            wait = max(0, client.ready_time - time)
            time = max(time, client.ready_time)

            # print(f"  Client {cid} | travel={travel:.1f} | arrival={time - wait:.1f} | wait={wait:.1f} | time={time:.1f} | window=[{client.ready_time}, {client.due_time}]")

            if time > client.due_time:
                # print(f"  ❌ Violation sur client {cid} : {time:.1f} > {client.due_time}")
                return False

            time += client.service_time
            prev = cid

        depot = inst.depot
        time += dm[prev][0]
        # print(f"  Retour dépôt | time={time:.1f} | due={depot.due_time}")

        if time > depot.due_time:
            # print(f"  ❌ Violation retour dépôt : {time:.1f} > {depot.due_time}")
            return False

        return True

    @property
    def is_feasible(self) -> bool:
        return self.is_capacity_feasible and self.is_time_feasible()
    
    