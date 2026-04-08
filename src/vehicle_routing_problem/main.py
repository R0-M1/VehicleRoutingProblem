from pathlib import Path

from vehicle_routing_problem.generator import RandomGenerator
from vehicle_routing_problem.tests.test_metaheuristic import TestMetaheuristic
from vehicle_routing_problem.tests.test_operator import TestOperator
from vehicle_routing_problem.utils.parser import VRPParser
from vehicle_routing_problem.visualization.visualizer import Visualizer
from vehicle_routing_problem.generator import RandomGenerator, GreedyGenerator
from vehicle_routing_problem.visualization.visualizer import Visualizer
from vehicle_routing_problem.metaheuristics.simulated_annealing import SimulatedAnnealing
from vehicle_routing_problem.export.datastorage import DataStorage
from vehicle_routing_problem.export.exporterCSV import CSVExporter

if __name__ == "__main__":
    print("--- Lancement du Projet VRPTW ---")

    project_root = Path(__file__).resolve().parents[2]
    filepath = project_root / "data" / "raw" / "data101.vrp"

    instance = VRPParser.parse(filepath)

    generator = GreedyGenerator(instance)
    solution = generator.generate()

    #init pour exporter
    export_required = True
    storage = DataStorage()

    print("\n--- Solution Initiale ---")
    print(f"Distance totale : {solution.total_distance:.2f} km")
    print(f"Nombre de véhicules : {solution.nb_vehicles}")


    TestMetaheuristic.test_simulated_annealing(instance, solution)


    print("\n--- Visualisation d'une route ---")

    route_index = 0  # choisir la route à afficher

    solution1 = generator.generate()
    solution2 = generator.generate()

    if export_required:
        csv_visitor = CSVExporter(filename="mon_export_vrp")
        storage.accept(csv_visitor)
    else:
        print("Export CSV désactivé par l'utilisateur.")

#     Visualizer.single_route(
#         solution,
#         instance,
#         route_idx=route_index,
#         title=f"Route {route_index + 1}",
#         show=True
#     )

#     Visualizer.compare_solutions(
#         solutions=[solution1, solution2, solution],
#         instance=instance,
#         titles=["solution1", "solution2", "solution3"],
#         figsize=(15, 6),
#         show=True
#     )

#     Visualizer.compare_routes(
#     routes=[
#         solution.routes[0],
#         solution.routes[1],
#         solution.routes[2]
#     ],
#     instance=instance,
#     titles=["Route 1", "Route 2", "Route 3"],
#     show=True
# )

tester = TestOperator(instance, solution)

# Test individual
# new_sol = tester.test_intra_exchange(0, 1, 3)
# new_sol = tester.test_intra_relocate(0, 1, 6)
# new_sol = tester.test_intra_reverse(0)
# new_sol = tester.test_intra_2opt(0, 1, 5)
# new_sol = tester.test_inter_relocate(0, 1, 1, 2)  # route1, client, route2, pos
# new_sol = tester.test_inter_exchange(0, 1, 1, 0)  # route1, client1, route2, client2
new_sol = tester.test_inter_cross_exchange(0, 1, 2, 1, 3, 4)  # routes et segments

# Tests « best » (trouvent la meilleure amélioration)
# best_sol = tester.test_best_intra_exchange(update_current_solution=True)
# best_sol = tester.test_best_inter_exchange(update_current_solution=True)