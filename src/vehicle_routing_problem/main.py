from pathlib import Path
from vehicle_routing_problem.utils.parser import VRPParser
from vehicle_routing_problem.visualization.Visualizer import Visualizer
from vehicle_routing_problem.generator import RandomGenerator, GreedyGenerator

if __name__ == "__main__":
    print("--- Lancement du Projet VRPTW ---")

    project_root = Path(__file__).resolve().parents[2]
    filepath = project_root / "data" / "raw" / "data101.vrp"

    instance = VRPParser.parse(filepath)

    generator = RandomGenerator(instance)
    solution = generator.generate()

    print("\n--- Solution Générée ---")
    print(f"Distance totale : {solution.total_distance:.2f} km")
    print(f"Nombre de véhicules : {solution.nb_vehicles}")
    
    # print("\n--- Étape 1 : Sans Time Windows ---")
    # sol_init = generate_random_solution(nodes, capacity, dist_matrix, use_tw=False)
    # print(f"Solution initiale : {sol_init.total_distance(dist_matrix):.2f} km avec {sol_init.total_vehicles()} véhicules.")

    print("\n--- Visualisation d'une route ---")

    route_index = 0  # choisir la route à afficher

    Visualizer.single_route(
        solution,
        instance,
        route_idx=route_index,
        title=f"Route {route_index + 1}",
        show=True
    )
