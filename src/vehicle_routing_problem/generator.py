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
        seed: int | None = None,
        fill_ratio: float = 1.0,
        check_time_windows: bool = False,
        rcl_size: int = 3,
    ):
        self.inst = instance
        self.max_vehicles = max_vehicles or instance.nb_clients // 3 + 1
        self.fill_ratio = max(0.0, min(1.0, fill_ratio))
        self._check_time_windows = check_time_windows
        self.rcl_size = max(1, rcl_size)  # 1 = greedy pur, >1 = randomisé
        self._rng = random.Random(seed)  # None → random à chaque appel

    def generate(self) -> Solution:
        customers = set(range(1, self.inst.nb_clients + 1))
        routes = []
        inst = self.inst
        capacity_threshold = inst.capacity * self.fill_ratio
        while customers:
            route_ids = []
            current_load = 0
            current_time = 0.0
            last_cid = 0
            while customers:
                if current_load >= capacity_threshold:
                    break
                # Construire la liste des candidats faisables avec leur distance
                candidates = []
                for cid in customers:
                    client = inst.clients[cid]
                    if current_load + client.demand > inst.capacity:
                        continue
                    travel = inst.dist_matrix[last_cid][cid]
                    if self._check_time_windows:
                        arrival = current_time + travel
                        service_start = max(arrival, client.ready_time)
                        if service_start > client.due_time:
                            continue
                        return_time = (service_start + client.service_time
                                       + inst.dist_matrix[cid][0])
                        if return_time > inst.depot.due_time:
                            continue
                    candidates.append((cid, travel))
                if not candidates:
                    break
                # Restricted Candidate List : trier par distance et choisir
                # aléatoirement parmi les rcl_size meilleurs
                candidates.sort(key=lambda x: x[1])
                rcl = candidates[:self.rcl_size]
                chosen_cid, _ = self._rng.choice(rcl)
                client = inst.clients[chosen_cid]
                travel = inst.dist_matrix[last_cid][chosen_cid]
                arrival = current_time + travel
                current_time = max(arrival, client.ready_time) + client.service_time
                current_load += client.demand
                route_ids.append(chosen_cid)
                customers.remove(chosen_cid)
                last_cid = chosen_cid
            if route_ids:
                routes.append(Route(route_ids, inst))
            elif customers:
                # Anti boucle infinie : force un client seul
                cid = next(iter(customers))
                routes.append(Route([cid], inst))
                customers.remove(cid)
        return Solution(routes)

class NearestNeighborGenerator:
    def __init__(self, instance: Instance):
        self.inst = instance

    def generate(self) -> Solution:
        inst = self.inst
        unvisited = [True] * (inst.nb_clients + 1)
        unvisited_count = inst.nb_clients
        routes = []

        while unvisited_count > 0:
            route_ids = []
            current_load = 0
            current_time = inst.depot.ready_time
            current_node = 0
            
            added = True
            while added and unvisited_count > 0:
                added = False
                best_client = -1
                best_arrival = 99999999.0
                
                for i in range(1, inst.nb_clients + 1):
                    if unvisited[i]:
                        client = inst.clients[i]
                        travel = inst.dist_matrix[current_node][i]
                        arrival = current_time + travel
                        
                        if arrival < client.ready_time:
                            arrival = client.ready_time
                        
                        can_add = True
                        if current_load + client.demand > inst.capacity:
                            can_add = False
                        if arrival > client.due_time:
                            can_add = False
                        
                        if can_add:
                            return_travel = inst.dist_matrix[i][0]
                            if arrival + client.service_time + return_travel > inst.depot.due_time:
                                can_add = False
                        
                        if can_add and arrival < best_arrival:
                            best_arrival = arrival
                            best_client = i
                
                if best_client != -1:
                    route_ids.append(best_client)
                    client = inst.clients[best_client]
                    current_load += client.demand
                    current_time = best_arrival + client.service_time
                    current_node = best_client
                    unvisited[best_client] = False
                    unvisited_count -= 1
                    added = True
            
            if route_ids:
                routes.append(Route(route_ids, inst))
            elif unvisited_count > 0:
                # Sécurité pour éviter boucle infinie si un client est impossible à livrer seul
                for i in range(1, inst.nb_clients + 1):
                    if unvisited[i]:
                        routes.append(Route([i], inst))
                        unvisited[i] = False
                        unvisited_count -= 1
                        break

        return Solution(routes)
