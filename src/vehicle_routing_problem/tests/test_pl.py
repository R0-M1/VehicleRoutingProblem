import time
from functools import wraps
from vehicle_routing_problem.pl.adaptateurpl import LinearProgrammingSolver
from vehicle_routing_problem.visualization.visualizer import Visualizer
from vehicle_routing_problem.export.datastorage import DataStorage

class TestPL:
    """
    Classe de test dédiée à la Programmation Linéaire (Solver Exact).
    Suit la même logique que TestMetaheuristic pour la cohérence du projet.
    """

    def timer(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            start = time.perf_counter()
            result = func(*args, **kwargs)
            end = time.perf_counter()
            print(f"L'exécution de [{func.__name__}] a duré {end - start:.4f} secondes")
            return result
        return wrapper

    @timer
    @staticmethod
    def test_solver_exact(instance, time_limit=60):
        """
        Lance le solveur PL sur une instance donnée.
        Le principe est identique aux métaheuristiques, mais sans solution initiale.
        """
        print(f"--- Test du Solver PL (Exact) sur {len(instance.clients)-1} clients ---")
        
        # Initialisation de l'adapter (Dakin / Simplexe via OR-Tools)
        solver = LinearProgrammingSolver(instance, time_limit=time_limit)
        
        solution_generee = False
        
        # On itère sur les solutions yieldées (généralement une seule pour la PL)
        for sol in solver.solve():
            solution_generee = True
            
            print(f"Résultat : Distance = {sol.total_distance:.2f} | Véhicules = {sol.nb_vehicles}")

            # Mise à jour des stats pour le rapport final (Question 5 & 6)
            DataStorage.update({
                "method": "Linear Programming",
                "nb_clients": len(instance.clients) - 1,
                "distance": round(sol.total_distance, 2),
                "nb_vehicles": sol.nb_vehicles,
                "time_limit": time_limit
            })

            # Visualisation du résultat
            Visualizer.update(
                sol, 
                instance, 
                title=f"PL Exact - {len(instance.clients)-1} clients",
                extra_stats=f"Distance optimale : {sol.total_distance:.2f}\nVéhicules : {sol.nb_vehicles}"
            )

        if not solution_generee:
            print(f"Attention : Aucune solution trouvée en {time_limit}s.")
            print("C'est normal si le nombre de clients est trop élevé (> 20).")

        Visualizer.keep_open()