from __future__ import annotations
import random
from vehicle_routing_problem.core.route import Route
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.instance import Instance


class RandomGenerator:
    """
    Générateur de solution initiale aléatoire pour VRPTW.
    Algorithme : affectation aléatoire des clients aux véhicules.
    """

    def __init__(self, instance: Instance, max_vehicles: int = 0, seed: int = 42, fill_ratio: float = 1.0):
        """
        Initialise le générateur aléatoire.

        Args:
            instance (Instance): Instance du problème VRPTW contenant les clients,
                la matrice de distances et la capacité des véhicules.
            max_vehicles (int, optional): Nombre maximum de véhicules autorisés.
                Si 0 (défaut), fixé automatiquement à 2 × nb_clients.
            seed (int, optional): Graine aléatoire pour la reproductibilité des résultats.
                Défaut : 42.
            fill_ratio (float, optional): Taux de remplissage cible des véhicules,
                compris entre 0.0 et 1.0.
                - 1.0 (défaut) : remplit chaque véhicule jusqu'à sa capacité maximale
                  avant de passer au suivant.
                - 0.5 : passe au véhicule suivant dès que 50 % de la capacité est atteinte,
                  produisant des routes plus courtes avec plus de véhicules.
                Valeur automatiquement clampée dans [0.0, 1.0].
        """
        self.inst = instance
        self.max_vehicles = max_vehicles or instance.nb_clients * 2
        self.fill_ratio = max(0.0, min(1.0, fill_ratio))
        random.seed(seed)

    def generate(self) -> Solution:
        """
        Génère une solution initiale aléatoire, potentiellement infaisable.

        Les clients sont d'abord mélangés aléatoirement, puis affectés séquentiellement
        aux véhicules. Un véhicule est fermé et un nouveau est ouvert dès que le seuil
        de remplissage (`fill_ratio × capacity`) est atteint ou qu'aucun client restant
        ne peut y être ajouté sans dépasser la capacité réelle.

        Les clients non affectés (si `max_vehicles` est atteint) sont placés dans des
        routes unitaires (un client par véhicule), ce qui peut produire une solution
        infaisable en nombre de véhicules.

        Returns:
            Solution: Solution générée (faisabilité en capacité non garantie si
                `fill_ratio` < 1.0 et clients de forte demande).

        Notes:
            La contrainte de capacité réelle (`instance.capacity`) n'est jamais violée
            au niveau d'une route individuelle. Seul le seuil de fermeture anticipée
            est contrôlé par `fill_ratio`.
        """
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
        """
        Génère une solution faisable en capacité (fenêtres temporelles ignorées).

        Appelle `generate()` de manière répétée jusqu'à obtenir une solution où
        tous les clients sont visités et toutes les routes respectent la contrainte
        de capacité. Les fenêtres temporelles ne sont pas vérifiées ici ; elles
        sont évaluées ultérieurement par l'évaluateur dédié.

        Args:
            max_attempts (int, optional): Nombre maximal de tentatives avant abandon.
                Défaut : 1000.

        Returns:
            Solution: Première solution faisable en capacité trouvée.

        Raises:
            RuntimeError: Si aucune solution faisable n'est trouvée après
                `max_attempts` tentatives.

        Notes:
            Avec un `fill_ratio` très bas, chaque véhicule transporte peu de clients,
            ce qui augmente la probabilité d'obtenir une solution faisable rapidement
            mais au prix d'un plus grand nombre de véhicules utilisés.
        """
        for attempt in range(max_attempts):
            sol = self.generate()
            if sol.all_clients_visited(self.inst):
                if all(route.is_capacity_feasible for route in sol.routes):
                    return sol
        raise RuntimeError(
            f"Impossible de générer solution faisable en {max_attempts} tentatives"
        )


class GreedyGenerator:
    """
    Générateur glouton de type « Plus Proche Voisin » (Nearest Neighbor).
    Produit une meilleure solution initiale que le générateur aléatoire,
    adaptée comme point de départ pour les métaheuristiques.
    """

    def __init__(self, instance: Instance, max_vehicles: int = 0, seed: int = 42, fill_ratio: float = 1.0):
        """
        Initialise le générateur glouton.

        Args:
            instance (Instance): Instance du problème VRPTW contenant les clients,
                la matrice de distances et la capacité des véhicules.
            max_vehicles (int, optional): Nombre maximum de véhicules autorisés.
                Si 0 (défaut), fixé automatiquement à nb_clients // 3 + 1.
            seed (int, optional): Graine aléatoire pour la reproductibilité.
                Défaut : 42. (Actuellement utilisé pour compatibilité future ;
                le générateur glouton est déterministe.)
            fill_ratio (float, optional): Taux de remplissage cible des véhicules,
                compris entre 0.0 et 1.0.
                - 1.0 (défaut) : remplit chaque véhicule au maximum avant de passer
                  au suivant, minimisant le nombre de véhicules utilisés.
                - 0.7 : ferme la route courante à 70 % de capacité, favorisant des
                  routes plus courtes et plus flexibles pour les fenêtres temporelles.
                Valeur automatiquement clampée dans [0.0, 1.0].
        """
        self.inst = instance
        self.max_vehicles = max_vehicles or instance.nb_clients // 3 + 1
        self.fill_ratio = max(0.0, min(1.0, fill_ratio))
        random.seed(seed)

    def generate(self) -> Solution:
        """
        Génère une solution gloutonne par la stratégie du plus proche voisin.

        À chaque étape, le client non encore visité le plus proche du dernier client
        ajouté (ou du dépôt en début de route) est sélectionné, sous réserve que
        sa demande ne dépasse pas la capacité restante du véhicule courant. La route
        est fermée dès que le seuil de remplissage (`fill_ratio × capacity`) est
        atteint ou qu'aucun client compatible n'existe plus.

        Les clients non affectés (si `max_vehicles` est atteint) sont abandonnés
        silencieusement ; il est recommandé de vérifier `sol.all_clients_visited()`
        après l'appel.

        Returns:
            Solution: Solution gloutonne construite. Le nombre de véhicules utilisés
                dépend de `fill_ratio` : un ratio faible produit plus de routes courtes,
                un ratio élevé minimise le nombre de véhicules.

        Notes:
            La complexité est O(n² × max_vehicles) où n est le nombre de clients,
            en raison du parcours linéaire des clients restants à chaque étape.
            Pour de grandes instances, envisager une structure de données spatiale
            (k-d tree) pour accélérer la recherche du plus proche voisin.
        """
        customers = set(range(1, self.inst.nb_clients + 1))
        routes = []
        capacity_threshold = self.inst.capacity * self.fill_ratio

        while customers and len(routes) < self.max_vehicles:
            route_ids = []
            current_load = 0
            last_cid = 0  # commence au dépôt

            while customers:
                if current_load >= capacity_threshold:
                    break

                best_cid = None
                best_dist = float('inf')

                for cid in customers:
                    if (self.inst.clients[cid].demand + current_load <= self.inst.capacity
                            and self.inst.dist_matrix[last_cid, cid] < best_dist):
                        best_cid = cid
                        best_dist = self.inst.dist_matrix[last_cid, cid]

                if best_cid is None:
                    break

                route_ids.append(best_cid)
                current_load += self.inst.clients[best_cid].demand
                customers.remove(best_cid)
                last_cid = best_cid

            if route_ids:
                routes.append(Route(route_ids, self.inst))

        return Solution(routes)