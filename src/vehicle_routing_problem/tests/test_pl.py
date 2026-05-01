import time
from functools import wraps
from vehicle_routing_problem.pl.adaptateurpl import LinearProgrammingSolver
from vehicle_routing_problem.visualization.visualizer import Visualizer
from vehicle_routing_problem.export.datastorage import DataStorage
import time
import copy
from vehicle_routing_problem.export.datastorage import DataStorage
from vehicle_routing_problem.export.exporterCSV import CSVExporter

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

    @staticmethod
    def run_bonus_question6(full_instance, sizes=[5, 10, 12, 15, 18, 20, 22,25,26,27,28,29,30]):
            """
            Répond au bonus : crée des jeux de données croissants, 
            stocke les résultats et les exporte en CSV.
            """
            print(f"\n{'='*50}\nLANCEMENT DU TEST DE LIMITE PL (QUESTION 6)\n{'='*50}")
            
            # On vide l'historique pour avoir un CSV propre dédié au bonus
            DataStorage.clear()

            for n in sizes:
                print(f"\n>>> Analyse pour {n} clients...")
                
                # 1. Création de l'instance temporaire (sous-ensemble)
                # On réutilise ta logique de découpage
                temp_clients = full_instance.clients[:n+1] 
                temp_instance = copy.deepcopy(full_instance)
                temp_instance.clients = temp_clients
                
                # 2. Exécution du solveur via ta méthode existante
                # On met un time_limit un peu plus long pour vraiment voir la limite
                start_perf = time.perf_counter()
                
                # On appelle directement ta méthode de test qui fait déjà le DataStorage.update
                TestPL.test_solver_exact(temp_instance, time_limit=100)
                
                end_perf = time.perf_counter()
                duration = end_perf - start_perf

                # 3. Ajout d'une info spécifique pour le CSV du bonus
                # (Si test_solver_exact ne l'a pas déjà fait avec les bonnes clés)
                # Note : DataStorage.update est déjà appelé dans test_solver_exact
                
                if duration > 105: # Si on a dépassé le temps limite + marge
                    print(f"Arrêt des tests : la limite de calcul est atteinte ({n} clients).")
                    break

            # 4. EXPORTATION AUTOMATIQUE EN CSV
            print("\nGénération du rapport CSV...")
            exporter = CSVExporter(filename="bonus_question6_PL")
            DataStorage.accept(exporter)