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

    def get_time_profile(self) -> list[dict]:
        profile = []
        depot = self._inst.depot
        current_time = depot.ready_time
        current_location = 0
        
        for client_id in self.client_ids:
            client = self._inst.clients[client_id]
            
            # Temps de voyage depuis le dernier point
            travel_time = self._inst.dist_matrix[current_location, client_id]
            arrival_time = current_time + travel_time
            
            # Début du service : max(heure d'arrivée, ready_time)
            service_start_time = max(arrival_time, client.ready_time)
            
            # Fin du service
            service_end_time = service_start_time + client.service_time
            
            profile.append({
                'client_id': client_id,
                'arrival_time': arrival_time,
                'service_start_time': service_start_time,
                'service_end_time': service_end_time
            })
            
            current_time = service_end_time
            current_location = client_id
        
        return profile

    @property
    def is_time_window_feasible(self) -> bool:
        profile = self.get_time_profile()
        for entry in profile:
            client = self._inst.clients[entry['client_id']]
            service_start = entry['service_start_time']
            
            if service_start < client.ready_time or service_start > client.due_time:
                return False
        
        return True

    def copy(self) -> Route:
        return Route(self.client_ids.copy(), self._inst)

    def __len__(self):
        return len(self.client_ids)

    def __repr__(self):
        seq = " → ".join(map(str, self.client_ids))
        return f"Route(0 → {seq} → 0  |  d={self.distance:.1f}  |  load={self.total_demand})"
