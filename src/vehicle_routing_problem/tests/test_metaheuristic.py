from vehicle_routing_problem.metaheuristics.local_search import LocalSearch
from vehicle_routing_problem.metaheuristics.tabu_search import TabuSearch
from vehicle_routing_problem.metaheuristics.simulated_annealing import SimulatedAnnealing
from vehicle_routing_problem.visualization.visualizer import Visualizer


class TestMetaheuristic:

    @staticmethod
    def test_local_search(instance, initial_solution):
        ls = LocalSearch(instance)
        print("Local Search:")
        for i, sol in enumerate(ls.solve(initial_solution)):
            print(f"Iteration {i}: {sol.total_distance:.2f}")
            Visualizer.update(sol, instance, title=f"Local Search - Itération {i}")
        
        Visualizer.keep_open()

    @staticmethod
    def test_tabu_search(instance, initial_solution):
        tabu = TabuSearch(instance, tabu_size=20)
        print("Tabu Search:")
        for i, sol in enumerate(tabu.solve(initial_solution)):
            print(f"Iteration {i}: {sol.total_distance:.2f}")
            Visualizer.update(sol, instance, title=f"Tabu Search - Itération {i}")
            if i >= 100:
                print(f"Limite {i} atteinte")
                break
        
        Visualizer.keep_open()

    @staticmethod
    def test_simulated_annealing(instance, initial_solution):
        sa = SimulatedAnnealing(instance, initial_temperature=1000.0, cooling_rate=0.995)
        print("Recuit Simulé:")
        for i, sol in enumerate(sa.solve(initial_solution)):
            print(f"Iteration {i}: {sol.total_distance:.2f}")
            Visualizer.update(sol, instance, title=f"Test Recuit Simulé - Itération {i}")
            if i >= 50:
                print(f"Limite {i} atteinte")
                break
        
        Visualizer.keep_open()
