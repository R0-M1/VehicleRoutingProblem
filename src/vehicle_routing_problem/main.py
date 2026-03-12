from pathlib import Path
from utils.parser import VRPParser
from vehicle_routing_problem.generator import RandomGenerator, GreedyGenerator
from visualization.plot_routes import plot_vrptw_solution

if __name__ == "__main__":
    print("--- Lancement du Projet VRPTW ---")

    project_root = Path(__file__).resolve().parents[2]
    filepath = project_root / "data" / "raw" / "data101.vrp"

    instance = VRPParser.parse(filepath)

    generator = RandomGenerator(instance)
    solution = generator.generate()

    fig = plot_vrptw_solution(solution, instance, title="Ma solution optimale")
    fig.show()  # Interactif dans Jupyter/VSCode
    fig.write_image("ma_solution.png")  # PNG haute qualité

    # print("\n--- Étape 1 : Sans Time Windows ---")
    # sol_init = generate_random_solution(nodes, capacity, dist_matrix, use_tw=False)
    # print(f"Solution initiale : {sol_init.total_distance(dist_matrix):.2f} km avec {sol_init.total_vehicles()} véhicules.")
