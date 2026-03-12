from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.instance import Instance
from vehicle_routing_problem.operators.intra_exchange import IntraExchange
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