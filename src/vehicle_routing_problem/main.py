from pathlib import Path
from src.vehicle_routing_problem.utils.parser import VRPParser

if __name__ == "__main__":
    print("--- Lancement du Projet VRPTW ---")

    project_root = Path(__file__).resolve().parents[2]
    filepath = project_root / "data" / "raw" / "data101.vrp"

    instance = VRPParser.parse(filepath)



    # print("\n--- Étape 1 : Sans Time Windows ---")
    # sol_init = generate_random_solution(nodes, capacity, dist_matrix, use_tw=False)
    # print(f"Solution initiale : {sol_init.total_distance(dist_matrix):.2f} km avec {sol_init.total_vehicles()} véhicules.")
