from __future__ import annotations
from typing import List
import random
from vehicle_routing_problem.core.route import Route
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.instance import Instance


class RandomGenerator:
    """
    Générateur de solution initiale aléatoire pour VRPTW.
    Algorithme : affectation aléatoire des clients aux véhicules.
    """

    def __init__(self, instance: Instance, max_vehicles: int = None, seed: int = 42):
        """
        Args:
            instance: Problème VRPTW
            max_vehicles: Nb max de véhicules (défaut: 2×nb_clients)
            seed: Pour reproductibilité
        """
        self.inst = instance
        self.max_vehicles = max_vehicles or instance.nb_clients * 2
        random.seed(seed)

    def generate(self) -> Solution:
        """Génère une solution aléatoire (peut être infaisable)."""
        # Étape 1 : liste des clients à visiter (IDs 1..n)
        customers = list(range(1, self.inst.nb_clients + 1))
        random.shuffle(customers)

        # Étape 2 : créer des routes vides
        routes = []

        # Étape 3 : affecter aléatoirement les clients aux véhicules
        vehicle_idx = 0
        while customers and vehicle_idx < self.max_vehicles:
            route_ids = []
            current_load = 0

            # Remplir le véhicule jusqu'à saturation ou fin clients
            while customers and current_load < self.inst.capacity:
                # Prendre le prochain client aléatoire
                cid = customers.pop(0)
                demand = self.inst.clients[cid].demand

                if current_load + demand <= self.inst.capacity:
                    route_ids.append(cid)
                    current_load += demand
                else:
                    # Client trop gros → nouveau véhicule
                    customers.insert(0, cid)  # remettre en tête
                    break

            if route_ids:  # véhicule non vide
                routes.append(Route(route_ids, self.inst))
                vehicle_idx += 1

        # Les clients restants → véhicules unitaires
        for cid in customers:
            routes.append(Route([cid], self.inst))

        return Solution(routes)

    def generate_feasible(self, max_attempts: int = 1000) -> Solution:
        """
        Génère une solution **faisable** en capacité (TW ignorées).
        Retente jusqu'à épuisement des tentatives.
        """
        for attempt in range(max_attempts):
            sol = self.generate()
            if sol.all_clients_visited(self.inst):
                # Vérifier capacité (TW plus tard dans l'évaluateur)
                if all(route.is_capacity_feasible for route in sol.routes):
                    return sol
        raise RuntimeError(f"Impossible de générer solution faisable en {max_attempts} tentatives")






class GreedyGenerator:
    """
    Générateur glouton "Nearest Neighbor" (meilleure init pour métaheuristiques).
    """

    def __init__(self, instance: Instance, max_vehicles: int = None, seed: int = 42):
        self.inst = instance
        self.max_vehicles = max_vehicles or instance.nb_clients // 3 + 1
        random.seed(seed)

    def generate(self) -> Solution:
        customers = set(range(1, self.inst.nb_clients + 1))
        routes = []

        while customers and len(routes) < self.max_vehicles:
            route_ids = []
            current_load = 0
            last_cid = 0  # commence au dépôt

            while customers:
                # Client le plus proche du dernier + capacité OK
                best_cid = None
                best_dist = float('inf')

                for cid in customers:
                    if (self.inst.clients[cid].demand + current_load <= self.inst.capacity
                            and self.inst.dist_matrix[last_cid, cid] < best_dist):
                        best_cid = cid
                        best_dist = self.inst.dist_matrix[last_cid, cid]

                if best_cid is None:  # plus de place
                    break

                route_ids.append(best_cid)
                current_load += self.inst.clients[best_cid].demand
                customers.remove(best_cid)
                last_cid = best_cid

            if route_ids:
                routes.append(Route(route_ids, self.inst))

        return Solution(routes)
