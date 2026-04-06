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

    def __init__(self, instance: Instance, max_vehicles: int = 0, seed: int = 42):
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

    def __init__(self, instance: Instance, max_vehicles: int = 0, seed: int = 42):
        self.inst = instance
        self.max_vehicles = max_vehicles or instance.nb_clients // 3 + 1
        random.seed(seed)

    def generate(self) -> Solution:
        customers = set(range(1, self.inst.nb_clients + 1))
        routes = []

        while customers and len(routes) < self.max_vehicles:
            route_ids = []
            current_load = 0
            current_time = 0.0 # Départ du dépôt au temps 0
            last_cid = 0

            while customers:
                best_cid = None
                best_score = float('inf')

                for cid in customers:
                    client = self.inst.clients[cid]
                    
                    # 1. Filtre Capacité
                    if current_load + client.demand > self.inst.capacity:
                        continue
                        
                    travel_time = self.inst.dist_matrix[last_cid, cid]
                    arrival_time = current_time + travel_time
                    
                    # 2. Filtre Fermeture Client (la livraison peut se terminer après, 
                    # mais il faut arriver et commencer le service avant la fin)
                    if arrival_time > client.due_time:
                        continue
                        
                    start_service_time = max(arrival_time, client.ready_time)
                    
                    # 3. Filtre Retour Dépôt
                    depot = self.inst.clients[0]
                    finish_service_time = start_service_time + client.service_time
                    return_travel_time = self.inst.dist_matrix[cid, 0]
                    
                    if finish_service_time + return_travel_time > depot.due_time:
                        continue
                        
                    # Si valide, on calcule le score (Earliest Start Time + petit poids pour la distance)
                    score = start_service_time + 0.1 * travel_time
                    
                    if score < best_score:
                        best_score = score
                        best_cid = cid

                if best_cid is None:  # Plus de candidats valables (plus de place ou de temps)
                    break

                # Application du choix
                best_client = self.inst.clients[best_cid]
                travel_time = self.inst.dist_matrix[last_cid, best_cid]
                
                # Mise à l'heure de la montre
                arrival_time = current_time + travel_time
                start_service_time = max(arrival_time, best_client.ready_time)
                current_time = start_service_time + best_client.service_time
                
                current_load += best_client.demand
                last_cid = best_cid
                route_ids.append(best_cid)
                customers.remove(best_cid)

            if route_ids:
                routes.append(Route(route_ids, self.inst))
            else:
                # Si aucun candidat dès le départ du dépôt, on arrête pour éviter une boucle infinie de camions vides
                break

        # Fallback de sécurité : on force chaque client oublié dans un camion pénalisé
        for cid in customers:
            routes.append(Route([cid], self.inst))

        return Solution(routes)
