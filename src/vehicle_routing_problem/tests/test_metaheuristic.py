from vehicle_routing_problem.metaheuristics.local_search import LocalSearch
from vehicle_routing_problem.metaheuristics.tabu_search import TabuSearch
from vehicle_routing_problem.metaheuristics.simulated_annealing import SimulatedAnnealing
from vehicle_routing_problem.visualization.visualizer import Visualizer
from vehicle_routing_problem.export.datastorage import DataStorage
from vehicle_routing_problem.export.exporterCSV import CSVExporter
import time
import datetime
from functools import wraps


class TestMetaheuristic:
    def timer(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            start = time.perf_counter()
            result = func(*args, **kwargs)
            end = time.perf_counter()
            print(f"La fonction [{func.__name__}] a mis {end - start:.4f} secondes")
            return result

        return wrapper

    @timer
    @staticmethod
    def test_local_search(instance, initial_solution):
        ls = LocalSearch(instance, n_neighbors=1000)
        print("Local Search:")
        for i, sol in enumerate(ls.solve(initial_solution)):
            print(f"Iteration {i}: {sol.total_distance:.2f}")
            Visualizer.update(sol, instance, title=f"Local Search - Itération {i}")

        Visualizer.keep_open()

    @timer
    @staticmethod
    def test_tabu_search(instance, initial_solution):
        tabu = TabuSearch(instance, tabu_size=200, n_neighbors=1000)
        print("Tabu Search:")
        for i, sol in enumerate(tabu.solve(initial_solution)):
            print(f"Iteration {i}: {sol.total_distance:.2f}")
            DataStorage.update({
                "iteration": i,
                "distance": round(sol.total_distance, 2),
                "nb_vehicles": sol.nb_vehicles,
                "tabu_size": len(tabu.tabu_list),
                "n_neighbors": tabu._n_neighbors
            })
            Visualizer.update(sol, instance, title=f"Tabu Search - Itération {i}",
                              extra_stats=f"Taille liste tabu : {len(tabu.tabu_list)}\n"
                                          f"Nb voisins calculés : {tabu._n_neighbors}")
            if i >= 1000:
                print(f"Limite {i} atteinte")
                break

        Visualizer.keep_open()

    @timer
    @staticmethod
    def test_simulated_annealing(instance, initial_solution):
        sa = SimulatedAnnealing(instance, initial_temperature=50, cooling_rate=0.9999, check_time_windows=True)
        print("Recuit Simulé:")
        for i, sol in enumerate(sa.solve(initial_solution)):
            # On lit simplement la propriété que l'on vient de créer dans sa !
            temp = sa.current_temperature
            print(f"Iteration {i}: {sol.total_distance:.2f} | Temp: {temp:.6f}")
            DataStorage.update({
                "iteration": i,
                "distance": round(sol.total_distance, 2),
                "temperature": round(temp, 2),
                "nb_vehicles": sol.nb_vehicles
            })
            # Visualizer.update(
            #     sol,
            #     instance,
            #     title=f"Test Recuit Simulé - Itération {i}",
            #     extra_stats=f"Température : {temp:.2f}"
            # )

            if i >= 1000:
                print(f"Limite {i} atteinte")
                break
