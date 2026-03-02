from pathlib import Path
from src.vehicle_routing_problem.parser import load_instance
from vehicle_routing_problem.generator import generate_random_solution
from vehicle_routing_problem.metaheuristics.recuit_simule import simulated_annealing

if __name__ == "__main__":
    print("--- Lancement du Projet VRPTW ---")

    project_root = Path(__file__).resolve().parents[2]
    filepath = project_root / "data" / "raw" / "data101.vrp"
    nodes, capacity, dist_matrix = load_instance(filepath)
    print(f"Fichier chargé : {len(nodes) - 1} clients. Capacité : {capacity}")

    print("\n--- Étape 1 : Sans Time Windows ---")
    sol_init = generate_random_solution(nodes, capacity, dist_matrix, use_tw=False)
    print(f"Solution initiale : {sol_init.total_distance(dist_matrix):.2f} km avec {sol_init.total_vehicles()} véhicules.")

    # 3. Amélioration avec le Recuit Simulé
    sol_opti = simulated_annealing(sol_init, capacity, dist_matrix, use_tw=False)
    print(f"Après Recuit Simulé : {sol_opti.total_distance(dist_matrix):.2f} km avec {sol_opti.total_vehicles()} véhicules.")