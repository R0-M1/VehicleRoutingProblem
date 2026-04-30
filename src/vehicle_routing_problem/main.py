from pathlib import Path

from vehicle_routing_problem.core.route import Route
from vehicle_routing_problem.generator import RandomGenerator
from vehicle_routing_problem.tests.test_metaheuristic import TestMetaheuristic
from vehicle_routing_problem.tests.test_operator import TestOperator
from vehicle_routing_problem.utils.parser import VRPParser
from vehicle_routing_problem.visualization.visualizer import Visualizer
from vehicle_routing_problem.generator import RandomGenerator, GreedyGenerator
from vehicle_routing_problem.visualization.visualizer import Visualizer
from vehicle_routing_problem.metaheuristics.simulated_annealing import SimulatedAnnealing
# Exemple d'appel
from vehicle_routing_problem.tests.test_pl import TestPL



if __name__ == "__main__":
    print("--- Lancement du Projet VRPTW ---")

    project_root = Path(__file__).resolve().parents[2]
    filepath = project_root / "data" / "raw" / "data101.vrp"

    instance = VRPParser.parse(filepath)

    generator = GreedyGenerator(instance, fill_ratio=1.0, check_time_windows=True) #TODO: avec timewindows True y'a un bug sur le remplissage. Genre j'ai toujours le meme nombre de routes.
    solution = generator.generate()

    print("\n--- Solution Initiale ---")
    print(f"Distance totale : {solution.total_distance:.2f} km")
    print(f"Nombre de véhicules : {solution.nb_vehicles}")
    
    # Visualisons tout de suite cette solution de départ pour voir à quoi elle ressemble
    Visualizer.visualize_solution(solution, instance, title="Solution Initiale Greedy VRPTW")
    
    TestMetaheuristic.test_simulated_annealing(instance, solution)


    print("\n--- Lancement du Solveur Exact (Adapter PL) ---")
    TestPL.test_solver_exact(instance, time_limit=45)
    
    Visualizer.keep_open()



    # print("\n--- Visualisation d'une route ---")
    #
    # route_index = 0  # choisir la route à afficher (0 = Route 1)
    #
    # solution1 = generator.generate()
    # solution2 = generator.generate()
    #
    # Visualizer.single_route(
    #     solution,
    #     instance,
    #     route_idx=route_index,
    #     title=f"Route {route_index + 1}",
    #     show=True
    # )

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

# tester = TestOperator(instance, solution)

# Test individual
# new_sol = tester.test_intra_exchange(0, 1, 3)
# new_sol = tester.test_intra_relocate(0, 1, 6)
# new_sol = tester.test_intra_reverse(0)
# new_sol = tester.test_intra_2opt(0, 1, 5)
# new_sol = tester.test_inter_relocate(0, 1, 1, 2)  # route1, client, route2, pos
# new_sol = tester.test_inter_exchange(0, 1, 1, 0)  # route1, client1, route2, client2
# new_sol = tester.test_inter_cross_exchange(0, 1, 2, 1, 3, 4)  # routes et segments

# Tests « best » (trouvent la meilleure amélioration)
# best_sol = tester.test_best_intra_exchange(update_current_solution=True)
# best_sol = tester.test_best_inter_exchange(update_current_solution=True)

# Test simple d'une route : dépôt → c2 → c1 → dépôt
# route_test0 = Route([2, 1], instance)

# Test inverse de la route : dépôt → c1 → c2 → dépôt
# route_test1 = Route([1, 2], instance)

# print("Route test :", route_test0)
# print("Route test :", route_test1)

# print("Faisabilité capacité :", route_test0.is_capacity_feasible)
# print("Faisabilité temps :", route_test0.is_time_feasible())
# print("Faisabilité globale :", route_test0.is_feasible)
#
# print("Faisabilité capacité :", route_test1.is_capacity_feasible)
# print("Faisabilité temps :", route_test1.is_time_feasible())
# print("Faisabilité globale :", route_test1.is_feasible)