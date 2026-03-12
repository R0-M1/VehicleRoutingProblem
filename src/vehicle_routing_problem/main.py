from pathlib import Path

from vehicle_routing_problem.generator import RandomGenerator
from vehicle_routing_problem.tests.test_metaheuristic import TestMetaheuristic
from vehicle_routing_problem.tests.test_operator import TestOperator
from vehicle_routing_problem.utils.parser import VRPParser
from vehicle_routing_problem.visualization.visualizer import Visualizer
from vehicle_routing_problem.generator import RandomGenerator, GreedyGenerator

if __name__ == "__main__":
    print("--- Lancement du Projet VRPTW ---")

    project_root = Path(__file__).resolve().parents[2]
    filepath = project_root / "data" / "raw" / "data101.vrp"

    instance = VRPParser.parse(filepath)

    generator = GreedyGenerator(instance)
    solution = generator.generate()

    print("\n--- Solution Initiale ---")
    print(f"Distance totale : {solution.total_distance:.2f} km")
    print(f"Nombre de véhicules : {solution.nb_vehicles}")


    TestMetaheuristic.test_tabu_search(instance, solution)


    print("\n--- Visualisation d'une route ---")

    route_index = 0  # choisir la route à afficher

    solution1 = generator.generate()
    solution2 = generator.generate()

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
    new_solution = tester.test_intra_exchange(0, 1, 3)