from __future__ import annotations
import random
from vehicle_routing_problem.core.route import Route
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.instance import Instance


class RandomGenerator:
    def __init__(
        self,
        instance: Instance,
        max_vehicles: int = 0,
        seed: int = 42,
        fill_ratio: float = 1.0,
        check_time_windows: bool = False,
    ):
        self.inst = instance
        self.max_vehicles = max_vehicles or instance.nb_clients * 2
        self.fill_ratio = max(0.0, min(1.0, fill_ratio))
        self._check_time_windows = check_time_windows
        random.seed(seed)

    def generate(self) -> Solution:
        customers = list(range(1, self.inst.nb_clients + 1))
        random.shuffle(customers)

        routes = []
        capacity_threshold = self.inst.capacity * self.fill_ratio
        vehicle_idx = 0

        while customers and vehicle_idx < self.max_vehicles:
            route_ids = []
            current_load = 0

            while customers and current_load < capacity_threshold:
                cid = customers.pop(0)
                demand = self.inst.clients[cid].demand

                if current_load + demand <= self.inst.capacity:
                    route_ids.append(cid)
                    current_load += demand
                    if current_load >= capacity_threshold:
                        break
                else:
                    customers.insert(0, cid)
                    break

            if route_ids:
                routes.append(Route(route_ids, self.inst))
                vehicle_idx += 1

        for cid in customers:
            routes.append(Route([cid], self.inst))

        return Solution(routes)

    def generate_feasible(self, max_attempts: int = 1000) -> Solution:
        for _ in range(max_attempts):
            sol = self.generate()
            if not sol.all_clients_visited(self.inst):
                continue
            if not all(r.is_capacity_feasible for r in sol.routes):
                continue
            if self._check_time_windows and not all(r.is_time_feasible() for r in sol.routes):
                continue
            return sol
        raise RuntimeError(
            f"Impossible de générer une solution faisable en {max_attempts} tentatives"
        )


class GreedyGenerator:
    def __init__(
        self,
        instance: Instance,
        max_vehicles: int = 0,
        seed: int = 42,
        fill_ratio: float = 1.0,
        check_time_windows: bool = False,
    ):
        self.inst = instance
        self.max_vehicles = max_vehicles or instance.nb_clients // 3 + 1
        self.fill_ratio = max(0.0, min(1.0, fill_ratio))
        self._check_time_windows = check_time_windows
        random.seed(seed)

    def generate(self) -> Solution:
        """
        Génère une solution gloutonne (plus proche voisin).
        Si check_time_windows=True, insère les clients triés par ready_time
        et vérifie les time windows à chaque insertion.
        Si check_time_windows=False, sélectionne uniquement par distance.
        """
        customers = set(range(1, self.inst.nb_clients + 1))
        routes = []
        inst = self.inst
        capacity_threshold = inst.capacity * self.fill_ratio

        while customers and len(routes) < self.max_vehicles:
            route_ids = []
            current_load = 0
            current_time = 0.0
            last_cid = 0

            while customers:
                if current_load >= capacity_threshold:
                    break

                if self._check_time_windows:
                    # Tri par ready_time, insertion du premier client faisable
                    sorted_candidates = sorted(
                        customers,
                        key=lambda cid: inst.clients[cid].ready_time
                    )
                    best_cid = None
                    for cid in sorted_candidates:
                        client = inst.clients[cid]
                        if current_load + client.demand > inst.capacity:
                            continue
                        travel = inst.dist_matrix[last_cid][cid]
                        arrival = current_time + travel
                        service_start = max(arrival, client.ready_time)
                        if service_start > client.due_time:
                            continue
                        return_time = service_start + client.service_time + inst.dist_matrix[cid][0]
                        if return_time > inst.depot.due_time:
                            continue
                        best_cid = cid
                        break
                else:
                    # Sélection par distance uniquement (plus proche voisin)
                    best_cid = None
                    best_dist = float('inf')
                    for cid in customers:
                        if (inst.clients[cid].demand + current_load <= inst.capacity
                                and inst.dist_matrix[last_cid, cid] < best_dist):
                            best_cid = cid
                            best_dist = inst.dist_matrix[last_cid, cid]

                if best_cid is None:
                    break

                client = inst.clients[best_cid]
                travel = inst.dist_matrix[last_cid][best_cid]
                arrival = current_time + travel
                current_time = max(arrival, client.ready_time) + client.service_time
                current_load += client.demand
                route_ids.append(best_cid)
                customers.remove(best_cid)
                last_cid = best_cid

            if route_ids:
                routes.append(Route(route_ids, inst))

        if customers and self._check_time_windows:
            raise RuntimeError(
                f"{len(customers)} clients non affectés — augmente max_vehicles "
                f"(actuel : {self.max_vehicles})"
            )

        # Clients restants en routes unitaires (fallback sans time windows)
        for cid in customers:
            routes.append(Route([cid], inst))

        return Solution(routes)