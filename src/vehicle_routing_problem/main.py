from pathlib import Path
from src.vehicle_routing_problem.parser import load_instance

if __name__ == "__main__":
    print("--- Lancement du Projet VRPTW ---")

    project_root = Path(__file__).resolve().parents[2]
    filepath = project_root / "data" / "raw" / "data101.vrp"
    nodes, capacity, dist_matrix = load_instance(filepath)
    print(f"Fichier chargé : {len(nodes) - 1} clients. Capacité : {capacity}")
