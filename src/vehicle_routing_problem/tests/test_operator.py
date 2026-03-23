from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.operators.intra_exchange import IntraExchange
from vehicle_routing_problem.operators.intra_relocate import IntraRelocate
from vehicle_routing_problem.operators.intra_reverse import IntraReverse
from vehicle_routing_problem.operators.intra_2opt import Intra2Opt
from vehicle_routing_problem.operators.inter_relocate import InterRelocate
from vehicle_routing_problem.operators.inter_exchange import InterExchange
from vehicle_routing_problem.operators.inter_cross_exchange import InterCrossExchange
from vehicle_routing_problem.visualization import visualizer


class TestOperator:
    def __init__(self, instance: Instance, solution: Solution):
        self.instance = instance
        self.solution = solution

    def print_solution_info(self, solution: Solution, title: str = "Solution") -> None:
        print(f"\n--- {title} ---")
        print(f"Distance totale : {solution.total_distance:.2f}")
        print(f"Nombre de véhicules : {solution.nb_vehicles}")

        for i, route in enumerate(solution.routes):
            print(
                f"Route {i} | clients={route.client_ids} | "
                f"charge={route.total_demand} | distance={route.distance:.2f}"
            )

# ============= INTRA EXCHANGE =============
    def test_intra_exchange_neighbors(self) -> list[IntraExchange]:
        """
        Génère tous les voisins IntraExchange possibles à partir
        de la solution courante.
        """
        generator = IntraExchange(self.instance, 0, 0, 0)
        neighbors = generator.generate_neighbors(self.solution)

        print("\n--- Génération des voisins IntraExchange ---")
        print(f"Nombre total de voisins générés : {len(neighbors)}")

        for idx, op in enumerate(neighbors[:10]):  # on affiche juste les 10 premiers
            print(
                f"Voisin {idx}: route_id={op.route_id}, "
                f"client1={op.client1}, client2={op.client2}"
            )

        return neighbors

    def test_intra_exchange(self, route_id: int, client1: int, client2: int):
        old_route = self.solution.routes[route_id].copy()

        changed_client_1 = old_route.client_ids[client1]
        changed_client_2 = old_route.client_ids[client2]

        op = IntraExchange(self.instance, route_id, client1, client2)
        new_solution = op.apply(self.solution)

        new_route = new_solution.routes[route_id]

        print(f"Route testée : {route_id}")
        print(f"Swap : positions {client1 + 1} et {client2 + 1}")
        print(f"Clients échangés : {changed_client_1} et {changed_client_2}")
        print(f"Distance avant : {self.solution.total_distance:.2f}")
        print(f"Distance après : {new_solution.total_distance:.2f}")

        visualizer.Visualizer.compare_routes(
            [old_route, new_route],
            self.instance,
            titles=["Avant", "Après"],
            highlight_clients=[changed_client_1, changed_client_2],
            suptitle="Intra Exchange",
            show=True
        )

        return new_solution

    def test_best_intra_exchange(self, update_current_solution: bool = False) -> Solution:
        """
        Génère tous les voisins IntraExchange, applique celui
        qui donne la meilleure distance totale.
        """
        print("\n=== TEST BEST INTRA EXCHANGE ===")
        self.print_solution_info(self.solution, "Solution initiale")

        generator = IntraExchange(self.instance, 0, 0, 0)
        neighbors = generator.generate_neighbors(self.solution)

        if not neighbors:
            print("Aucun voisin généré.")
            return self.solution

        best_solution = self.solution
        best_operator = None
        best_distance = self.solution.total_distance

        for op in neighbors:
            candidate = op.apply(self.solution)

            if candidate.total_distance < best_distance:
                best_distance = candidate.total_distance
                best_solution = candidate
                best_operator = op

        if best_operator is None:
            print("Aucune amélioration trouvée avec IntraExchange.")
            return self.solution

        print(
            f"\nMeilleur échange trouvé : "
            f"route_id={best_operator.route_id}, "
            f"client1={best_operator.client1}, "
            f"client2={best_operator.client2}"
        )
        print(f"Distance avant : {self.solution.total_distance:.2f}")
        print(f"Distance après : {best_solution.total_distance:.2f}")
        print(f"Gain : {self.solution.total_distance - best_solution.total_distance:.2f}")

        self.print_solution_info(best_solution, "Meilleure solution trouvée")

        if update_current_solution:
            self.solution = best_solution

        return best_solution

    # ============= INTRA RELOCATE =============
    def test_intra_relocate(self, route_id: int, client1: int, client2: int):
        """Déplace un client d'une position à une autre dans la même route."""
        old_route = self.solution.routes[route_id].copy()
        moved_client = old_route.client_ids[client1]

        op = IntraRelocate(self.instance, route_id, client1, client2)
        new_solution = op.apply(self.solution)
        new_route = new_solution.routes[route_id]

        print(f"\n=== INTRA RELOCATE ===")
        print(f"Route testée : {route_id}")
        print(f"Client déplacé : {moved_client} (pos {client1} → pos {client2})")
        print(f"Distance avant : {self.solution.total_distance:.2f}")
        print(f"Distance après : {new_solution.total_distance:.2f}")
        print(f"Gain : {self.solution.total_distance - new_solution.total_distance:.2f}")

        visualizer.Visualizer.compare_routes(
            [old_route, new_route],
            self.instance,
            titles=["Avant", "Après"],
            highlight_clients=[moved_client],
            suptitle="Intra Relocate",
            show=True
        )

        return new_solution

    def test_best_intra_relocate(self, update_current_solution: bool = False) -> Solution:
        """Trouve le meilleur déplacement intra-route."""
        print("\n=== TEST BEST INTRA RELOCATE ===")
        self.print_solution_info(self.solution, "Solution initiale")

        generator = IntraRelocate(self.instance, 0, 0, 0)
        neighbors = generator.generate_neighbors(self.solution)

        if not neighbors:
            print("Aucun voisin généré.")
            return self.solution

        best_solution = self.solution
        best_operator = None
        best_distance = self.solution.total_distance

        for op in neighbors:
            candidate = op.apply(self.solution)
            if candidate.total_distance < best_distance:
                best_distance = candidate.total_distance
                best_solution = candidate
                best_operator = op

        if best_operator is None:
            print("Aucune amélioration trouvée avec IntraRelocate.")
            return self.solution

        print(f"\nMeilleur relocate trouvé : route={best_operator.route_id}, pos {best_operator.client1} → {best_operator.client2}")
        print(f"Distance avant : {self.solution.total_distance:.2f}")
        print(f"Distance après : {best_solution.total_distance:.2f}")
        print(f"Gain : {self.solution.total_distance - best_solution.total_distance:.2f}")

        self.print_solution_info(best_solution, "Meilleure solution trouvée")

        if update_current_solution:
            self.solution = best_solution

        return best_solution

    # ============= INTRA REVERSE =============
    def test_intra_reverse(self, route_id: int):
        """Inverse complètement une route."""
        old_route = self.solution.routes[route_id].copy()
        old_order = old_route.client_ids.copy()

        op = IntraReverse(self.instance, route_id)
        new_solution = op.apply(self.solution)
        new_route = new_solution.routes[route_id]

        print(f"\n=== INTRA REVERSE ===")
        print(f"Route testée : {route_id}")
        print(f"Avant : {old_order}")
        print(f"Après : {new_route.client_ids}")
        print(f"Distance avant : {self.solution.total_distance:.2f}")
        print(f"Distance après : {new_solution.total_distance:.2f}")
        print(f"Gain : {self.solution.total_distance - new_solution.total_distance:.2f}")

        visualizer.Visualizer.compare_routes(
            [old_route, new_route],
            self.instance,
            titles=["Avant", "Après (inversé)"],
            suptitle="Intra Reverse",
            show=True
        )

        return new_solution

    def test_best_intra_reverse(self, update_current_solution: bool = False) -> Solution:
        """Teste l'inversion de chaque route et gardela meilleure."""
        print("\n=== TEST BEST INTRA REVERSE ===")
        self.print_solution_info(self.solution, "Solution initiale")

        generator = IntraReverse(self.instance, 0)
        neighbors = generator.generate_neighbors(self.solution)

        if not neighbors:
            print("Aucun voisin généré.")
            return self.solution

        best_solution = self.solution
        best_operator = None
        best_distance = self.solution.total_distance

        for op in neighbors:
            candidate = op.apply(self.solution)
            if candidate.total_distance < best_distance:
                best_distance = candidate.total_distance
                best_solution = candidate
                best_operator = op

        if best_operator is None:
            print("Aucune amélioration trouvée avec IntraReverse.")
            return self.solution

        print(f"\nMeilleur reverse trouvé : route_id={best_operator.route_id}")
        print(f"Distance avant : {self.solution.total_distance:.2f}")
        print(f"Distance après : {best_solution.total_distance:.2f}")
        print(f"Gain : {self.solution.total_distance - best_solution.total_distance:.2f}")

        self.print_solution_info(best_solution, "Meilleure solution trouvée")

        if update_current_solution:
            self.solution = best_solution

        return best_solution

    # ============= INTRA 2-OPT =============
    def test_intra_2opt(self, route_id: int, client1: int, client2: int):
        """Inverse une sous-séquence entre deux positions."""
        old_route = self.solution.routes[route_id].copy()
        old_subsequence = old_route.client_ids[client1:client2+1].copy()

        op = Intra2Opt(self.instance, route_id, client1, client2)
        new_solution = op.apply(self.solution)
        new_route = new_solution.routes[route_id]
        new_subsequence = new_route.client_ids[client1:client2+1]

        print(f"\n=== INTRA 2-OPT ===")
        print(f"Route testée : {route_id}")
        print(f"Inversion positions {client1} à {client2}")
        print(f"Avant : {old_subsequence}")
        print(f"Après : {list(reversed(old_subsequence))}")
        print(f"Distance avant : {self.solution.total_distance:.2f}")
        print(f"Distance après : {new_solution.total_distance:.2f}")
        print(f"Gain : {self.solution.total_distance - new_solution.total_distance:.2f}")

        visualizer.Visualizer.compare_routes(
            [old_route, new_route],
            self.instance,
            titles=["Avant", "Après (2-opt)"],
            suptitle="Intra 2-Opt",
            show=True
        )

        return new_solution

    def test_best_intra_2opt(self, update_current_solution: bool = False) -> Solution:
        """Trouve le meilleur 2-opt possible."""
        print("\n=== TEST BEST INTRA 2-OPT ===")
        self.print_solution_info(self.solution, "Solution initiale")

        generator = Intra2Opt(self.instance, 0, 0, 0)
        neighbors = generator.generate_neighbors(self.solution)

        if not neighbors:
            print("Aucun voisin généré.")
            return self.solution

        best_solution = self.solution
        best_operator = None
        best_distance = self.solution.total_distance

        for op in neighbors:
            candidate = op.apply(self.solution)
            if candidate.total_distance < best_distance:
                best_distance = candidate.total_distance
                best_solution = candidate
                best_operator = op

        if best_operator is None:
            print("Aucune amélioration trouvée avec Intra2Opt.")
            return self.solution

        print(f"\nMeilleur 2-opt trouvé : route={best_operator.route_id}, pos {best_operator.client1} à {best_operator.client2}")
        print(f"Distance avant : {self.solution.total_distance:.2f}")
        print(f"Distance après : {best_solution.total_distance:.2f}")
        print(f"Gain : {self.solution.total_distance - best_solution.total_distance:.2f}")

        self.print_solution_info(best_solution, "Meilleure solution trouvée")

        if update_current_solution:
            self.solution = best_solution

        return best_solution

    # ============= INTER RELOCATE =============
    def test_inter_relocate(self, route1_id: int, client: int, route2_id: int, insert_pos: int):
        """Déplace un client d'une route à une autre."""
        old_route1 = self.solution.routes[route1_id].copy()
        old_route2 = self.solution.routes[route2_id].copy()
        moved_client = old_route1.client_ids[client]

        op = InterRelocate(self.instance, route1_id, route2_id, client, insert_pos)
        new_solution = op.apply(self.solution)
        new_route1 = new_solution.routes[route1_id]
        new_route2 = new_solution.routes[route2_id]

        print(f"\n=== INTER RELOCATE ===")
        print(f"Client déplacé : {moved_client} de route {route1_id} à route {route2_id}")
        print(f"Position d'insertion dans route {route2_id} : {insert_pos}")
        print(f"Distance avant : {self.solution.total_distance:.2f}")
        print(f"Distance après : {new_solution.total_distance:.2f}")
        print(f"Gain : {self.solution.total_distance - new_solution.total_distance:.2f}")

        visualizer.Visualizer.compare_routes(
            [old_route1, new_route1, old_route2, new_route2],
            self.instance,
            titles=["Route1 Avant", "Route1 Après", "Route2 Avant", "Route2 Après"],
            highlight_clients=[moved_client],
            suptitle="Inter Relocate",
            show=True
        )

        return new_solution

    def test_best_inter_relocate(self, update_current_solution: bool = False) -> Solution:
        """Trouve le meilleur relocate inter-route."""
        print("\n=== TEST BEST INTER RELOCATE ===")
        self.print_solution_info(self.solution, "Solution initiale")

        generator = InterRelocate(self.instance, 0, 0, 0, 0)
        neighbors = generator.generate_neighbors(self.solution)

        if not neighbors:
            print("Aucun voisin généré.")
            return self.solution

        best_solution = self.solution
        best_operator = None
        best_distance = self.solution.total_distance

        for op in neighbors:
            candidate = op.apply(self.solution)
            if candidate.total_distance < best_distance:
                best_distance = candidate.total_distance
                best_solution = candidate
                best_operator = op

        if best_operator is None:
            print("Aucune amélioration trouvée avec InterRelocate.")
            return self.solution

        print(f"\nMeilleur relocate inter trouvé : client de route {best_operator.route1_id} vers route {best_operator.route2_id}")
        print(f"Distance avant : {self.solution.total_distance:.2f}")
        print(f"Distance après : {best_solution.total_distance:.2f}")
        print(f"Gain : {self.solution.total_distance - best_solution.total_distance:.2f}")

        self.print_solution_info(best_solution, "Meilleure solution trouvée")

        if update_current_solution:
            self.solution = best_solution

        return best_solution

    # ============= INTER EXCHANGE =============
    def test_inter_exchange(self, route1_id: int, client1: int, route2_id: int, client2: int):
        """Échange un client d'une route avec un client d'une autre route."""
        old_route1 = self.solution.routes[route1_id].copy()
        old_route2 = self.solution.routes[route2_id].copy()
        client1_id = old_route1.client_ids[client1]
        client2_id = old_route2.client_ids[client2]

        op = InterExchange(self.instance, route1_id, route2_id, client1, client2)
        new_solution = op.apply(self.solution)
        new_route1 = new_solution.routes[route1_id]
        new_route2 = new_solution.routes[route2_id]

        print(f"\n=== INTER EXCHANGE ===")
        print(f"Échange : client {client1_id} de route {route1_id} ↔ client {client2_id} de route {route2_id}")
        print(f"Distance avant : {self.solution.total_distance:.2f}")
        print(f"Distance après : {new_solution.total_distance:.2f}")
        print(f"Gain : {self.solution.total_distance - new_solution.total_distance:.2f}")

        visualizer.Visualizer.compare_routes(
            [old_route1, new_route1, old_route2, new_route2],
            self.instance,
            titles=["Route1 Avant", "Route1 Après", "Route2 Avant", "Route2 Après"],
            highlight_clients=[client1_id, client2_id],
            suptitle="Inter Exchange",
            show=True
        )

        return new_solution

    def test_best_inter_exchange(self, update_current_solution: bool = False) -> Solution:
        """Trouve le meilleur échange inter-route."""
        print("\n=== TEST BEST INTER EXCHANGE ===")
        self.print_solution_info(self.solution, "Solution initiale")

        generator = InterExchange(self.instance, 0, 0, 0, 0)
        neighbors = generator.generate_neighbors(self.solution)

        if not neighbors:
            print("Aucun voisin généré.")
            return self.solution

        best_solution = self.solution
        best_operator = None
        best_distance = self.solution.total_distance

        for op in neighbors:
            candidate = op.apply(self.solution)
            if candidate.total_distance < best_distance:
                best_distance = candidate.total_distance
                best_solution = candidate
                best_operator = op

        if best_operator is None:
            print("Aucune amélioration trouvée avec InterExchange.")
            return self.solution

        print(f"\nMeilleur échange inter trouvé : route {best_operator.route1_id} ↔ route {best_operator.route2_id}")
        print(f"Distance avant : {self.solution.total_distance:.2f}")
        print(f"Distance après : {best_solution.total_distance:.2f}")
        print(f"Gain : {self.solution.total_distance - best_solution.total_distance:.2f}")

        self.print_solution_info(best_solution, "Meilleure solution trouvée")

        if update_current_solution:
            self.solution = best_solution

        return best_solution

    # ============= INTER CROSS EXCHANGE =============
    def test_inter_cross_exchange(self, route1_id: int, start1: int, end1: int, route2_id: int, start2: int, end2: int):
        """Échange des segments entre deux routes."""
        old_route1 = self.solution.routes[route1_id].copy()
        old_route2 = self.solution.routes[route2_id].copy()
        seg1 = old_route1.client_ids[start1:end1+1]
        seg2 = old_route2.client_ids[start2:end2+1]

        op = InterCrossExchange(self.instance, route1_id, route2_id, start1, end1, start2, end2)
        new_solution = op.apply(self.solution)

        if new_solution == self.solution:
            print(f"\n=== INTER CROSS EXCHANGE ===")
            print("Opération invalide (capacité dépassée ou indices invalides).")
            return new_solution

        new_route1 = new_solution.routes[route1_id]
        new_route2 = new_solution.routes[route2_id]

        print(f"\n=== INTER CROSS EXCHANGE ===")
        print(f"Segments échangés :")
        print(f"  Route {route1_id} : positions {start1}-{end1} = {seg1}")
        print(f"  Route {route2_id} : positions {start2}-{end2} = {seg2}")
        print(f"Distance avant : {self.solution.total_distance:.2f}")
        print(f"Distance après : {new_solution.total_distance:.2f}")
        print(f"Gain : {self.solution.total_distance - new_solution.total_distance:.2f}")

        visualizer.Visualizer.compare_routes(
            [old_route1, new_route1, old_route2, new_route2],
            self.instance,
            titles=["Route1 Avant", "Route1 Après", "Route2 Avant", "Route2 Après"],
            suptitle="Inter Cross Exchange",
            show=True
        )

        return new_solution

    def test_best_inter_cross_exchange(self, update_current_solution: bool = False) -> Solution:
        """Trouve le meilleur cross-exchange inter-route (limité pour performance)."""
        print("\n=== TEST BEST INTER CROSS EXCHANGE ===")
        self.print_solution_info(self.solution, "Solution initiale")

        generator = InterCrossExchange(self.instance, 0, 0, 0, 0, 0)
        neighbors = generator.generate_neighbors(self.solution)

        if not neighbors:
            print("Aucun voisin généré.")
            return self.solution

        best_solution = self.solution
        best_operator = None
        best_distance = self.solution.total_distance

        # Limiter le nombre de voisins testés pour performance (InterCrossExchange génère beaucoup de voisins)
        sample_size = min(500, len(neighbors))
        import random
        sampled_neighbors = random.sample(neighbors, sample_size)

        for op in sampled_neighbors:
            candidate = op.apply(self.solution)
            if candidate.total_distance < best_distance:
                best_distance = candidate.total_distance
                best_solution = candidate
                best_operator = op

        if best_operator is None:
            print(f"Aucune amélioration trouvée avec InterCrossExchange ({sample_size} voisins testés).")
            return self.solution

        print(f"\nMeilleur cross-exchange trouvé : route {best_operator.route1_id} ↔ route {best_operator.route2_id}")
        print(f"Distance avant : {self.solution.total_distance:.2f}")
        print(f"Distance après : {best_solution.total_distance:.2f}")
        print(f"Gain : {self.solution.total_distance - best_solution.total_distance:.2f}")

        self.print_solution_info(best_solution, "Meilleure solution trouvée")

        if update_current_solution:
            self.solution = best_solution

        return best_solution
