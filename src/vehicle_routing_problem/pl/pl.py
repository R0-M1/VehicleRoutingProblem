"""
VRPTW - Vehicle Routing Problem with Time Windows
Résolution exacte par ILP avec OR-Tools (CP-SAT / MPSolver)
Question 6 du projet : analyse de la limite de résolvabilité
"""

import math
import time
import random
import matplotlib.pyplot as plt
from ortools.linear_solver import pywraplp

from vehicle_routing_problem.core.client import Client
from vehicle_routing_problem.utils.parser import VRPParser


# ─────────────────────────────────────────────
#  STRUCTURES DE DONNÉES
# ─────────────────────────────────────────────


class VRPTWInstance:
    def __init__(self, clients, capacity, num_vehicles=None):
        """
        clients      : liste de Client (index 0 = dépôt)
        capacity     : capacité C de chaque véhicule
        num_vehicles : nombre de véhicules (si None → estimé automatiquement)
        """
        self.clients      = clients
        self.capacity     = capacity
        self.n            = len(clients) - 1       # nombre de clients (sans dépôt)
        self.num_vehicles = num_vehicles or self.n  # borne sup : 1 véhicule/client

    def distance(self, i, j):
        """Distance euclidienne entre les clients i et j."""
        return self.clients[i].distance_to(self.clients[j])

    def travel_time(self, i, j):
        """Temps de trajet = distance euclidienne (vitesse = 1)."""
        return self.distance(i, j)


# ─────────────────────────────────────────────
#  PARSEUR DE FICHIER (format Solomon standard)
# ─────────────────────────────────────────────

def parse_solomon(filepath):
    """
    Parse un fichier au format Solomon :
    Ligne 0 : nom de l'instance
    Ligne 4 : VEHICLE  NUMBER  CAPACITY
    Ligne 5 : valeurs num_vehicles capacity
    Ligne 9+ : CUST  XCOORD  YCOORD  DEMAND  READY_TIME  DUE_DATE  SERVICE
    """
    clients = []
    capacity = 0
    num_vehicles = 0

    with open(filepath, 'r') as f:
        lines = [l.strip() for l in f if l.strip()]

    # Cherche la ligne de capacité
    for i, line in enumerate(lines):
        if 'VEHICLE' in line.upper():
            parts = lines[i + 1].split()
            num_vehicles = int(parts[0])
            capacity     = int(parts[1])
        if 'CUST' in line.upper():
            # Les clients commencent à la ligne suivante
            for line in lines[i + 1:]:
                parts = line.split()
                if len(parts) >= 7:
                    clients.append(Client(
                        id           = int(parts[0]),
                        name         = parts[0],
                        x            = float(parts[1]),
                        y            = float(parts[2]),
                        demand       = int(parts[3]),
                        ready_time   = int(parts[4]),
                        due_time     = int(parts[5]),
                        service_time = int(parts[6])
                    ))
            break

    return VRPTWInstance(clients, capacity, num_vehicles)


# ─────────────────────────────────────────────
#  GÉNÉRATEUR DE JEU DE DONNÉES ALÉATOIRE
#  (pour la question 6 : instances de taille croissante)
# ─────────────────────────────────────────────

def generate_random_instance(n_clients, capacity=200, seed=42,
                              grid_size=100, max_demand=30,
                              time_horizon=1000, tw_width=200):
    """
    Génère une instance VRPTW aléatoire avec n_clients clients.

    Paramètres
    ----------
    n_clients    : nombre de clients (hors dépôt)
    capacity     : capacité de chaque véhicule
    seed         : graine aléatoire
    grid_size    : taille de la grille (coordonnées dans [0, grid_size])
    max_demand   : demande max par client
    time_horizon : horizon temporel total (b_0 du dépôt)
    tw_width     : largeur moyenne des fenêtres de temps
    """
    random.seed(seed)

    # Dépôt au centre
    depot = Client(0, "depot", grid_size/2, grid_size/2,
                   demand=0,
                   ready_time=0, due_time=time_horizon,
                   service_time=0)
    clients = [depot]

    for i in range(1, n_clients + 1):
        x = random.uniform(0, grid_size)
        y = random.uniform(0, grid_size)
        demand = random.randint(1, max_demand)

        # Fenêtre de temps : ouverture aléatoire, fermeture = ouverture + largeur
        ready = random.randint(0, time_horizon - tw_width)
        due   = ready + random.randint(tw_width // 2, tw_width)
        due   = min(due, time_horizon)
        service = random.randint(5, 15)

        clients.append(Client(i, f"c{i}", x, y, demand, ready, due, service))

    return VRPTWInstance(clients, capacity)


def scale_instance(base_instance, n_clients, seed=0):
    """
    Construit une instance de taille n_clients à partir d'une instance de base
    en perturbant les coordonnées existantes (pour la question 6).
    """
    random.seed(seed)
    base_clients = base_instance.clients[1:]  # sans dépôt
    new_clients = [base_instance.clients[0]]  # garde le dépôt

    for i in range(1, n_clients + 1):
        base = random.choice(base_clients)
        noise_x = random.uniform(-5, 5)
        noise_y = random.uniform(-5, 5)
        new_clients.append(Client(
            id           = i,
            name         = f"c{i}",
            x            = base.x + noise_x,
            y            = base.y + noise_y,
            demand       = base.demand,
            ready_time   = base.ready_time,
            due_time     = base.due_time,
            service_time = base.service_time
        ))

    return VRPTWInstance(new_clients, base_instance.capacity)


# ─────────────────────────────────────────────
#  SOLVER ILP (OR-Tools SCIP)
# ─────────────────────────────────────────────

def solve_vrptw_ilp(instance, time_limit_seconds=300, with_time_windows=True):
    """
    Résout le VRPTW par ILP avec OR-Tools (SCIP).

    Retourne un dictionnaire avec :
    - status       : 'OPTIMAL', 'FEASIBLE', 'INFEASIBLE', 'TIMEOUT'
    - objective    : valeur de la fonction objectif (distance totale)
    - routes       : liste des routes par véhicule
    - solve_time   : temps de résolution en secondes
    - num_vehicles : nombre de véhicules utilisés
    """
    inst = instance
    N    = list(range(len(inst.clients)))   # 0..n (0 = dépôt)
    K    = list(range(inst.num_vehicles))   # véhicules
    n    = inst.n

    # ── Précalcul des distances et temps de trajet ──
    dist = [[inst.distance(i, j)    for j in N] for i in N]
    trav = [[inst.travel_time(i, j) for j in N] for i in N]

    # ── Big-M serré par paire (i,j) ──
    def big_M(i, j):
        ci = inst.clients[i]
        cj = inst.clients[j]
        return ci.due_time + ci.service_time + trav[i][j] - cj.ready_time

    # ── Création du solver ──
    solver = pywraplp.Solver.CreateSolver('SCIP')
    if not solver:
        raise RuntimeError("Solver SCIP non disponible")
    solver.SetTimeLimit(time_limit_seconds * 1000)  # en millisecondes

    # ────────────────────────────────
    #  VARIABLES
    # ────────────────────────────────

    # x[i][j][k] = 1 si véhicule k va de i à j
    x = {}
    for i in N:
        for j in N:
            if i == j:
                continue
            for k in K:
                x[i, j, k] = solver.BoolVar(f'x_{i}_{j}_{k}')

    # T[i][k] = heure d'arrivée du véhicule k au client i
    T = {}
    for i in N:
        for k in K:
            T[i, k] = solver.NumVar(0.0, solver.infinity(), f'T_{i}_{k}')

    # ────────────────────────────────
    #  FONCTION OBJECTIF
    # ────────────────────────────────
    obj = solver.Objective()
    for i in N:
        for j in N:
            if i == j:
                continue
            for k in K:
                obj.SetCoefficient(x[i, j, k], dist[i][j])
    obj.SetMinimization()

    # ────────────────────────────────
    #  CONTRAINTES
    # ────────────────────────────────

    # ① Chaque client visité exactement une fois
    for i in N:
        if i == 0:
            continue
        ct = solver.Constraint(1, 1, f'visit_{i}')
        for k in K:
            for j in N:
                if j != i:
                    ct.SetCoefficient(x[i, j, k], 1)

    # ② Conservation du flot
    for i in N:
        if i == 0:
            continue
        for k in K:
            ct = solver.Constraint(0, 0, f'flow_{i}_{k}')
            for j in N:
                if j != i:
                    ct.SetCoefficient(x[i, j, k],  1)   # sortant
                    ct.SetCoefficient(x[j, i, k], -1)   # entrant

    # ③ Chaque véhicule part du dépôt au plus une fois
    for k in K:
        ct = solver.Constraint(0, 1, f'depart_{k}')
        for j in N:
            if j != 0:
                ct.SetCoefficient(x[0, j, k], 1)

    # ④ Chaque véhicule revient au dépôt au plus une fois
    for k in K:
        ct = solver.Constraint(0, 1, f'retour_{k}')
        for i in N:
            if i != 0:
                ct.SetCoefficient(x[i, 0, k], 1)

    # ⑤ Contrainte de capacité
    for k in K:
        ct = solver.Constraint(0, inst.capacity, f'cap_{k}')
        for i in N:
            if i == 0:
                continue
            demand_i = inst.clients[i].demand
            for j in N:
                if j != i:
                    ct.SetCoefficient(x[i, j, k], demand_i)

    if with_time_windows:
        # ⑥ Fenêtres de temps
        for i in N:
            for k in K:
                ci = inst.clients[i]
                solver.Add(T[i, k] >= ci.ready_time)
                solver.Add(T[i, k] <= ci.due_time)

        # ⑦ Cohérence temporelle avec Big-M serré
        for i in N:
            for j in N:
                if i == j:
                    continue
                M_ij = big_M(i, j)
                if M_ij <= 0:
                    continue
                ci = inst.clients[i]
                for k in K:
                    # T[j,k] >= T[i,k] + s_i + t_ij - M_ij*(1 - x[i,j,k])
                    # ↔ T[j,k] - T[i,k] + M_ij * x[i,j,k] >= s_i + t_ij - M_ij + M_ij
                    # Réécrit : T[j,k] - T[i,k] + M_ij * x[i,j,k] >= s_i + t_ij
                    ct = solver.Constraint(
                        ci.service_time + trav[i][j] - M_ij,
                        solver.infinity(),
                        f'time_{i}_{j}_{k}'
                    )
                    ct.SetCoefficient(T[j, k],      1)
                    ct.SetCoefficient(T[i, k],     -1)
                    ct.SetCoefficient(x[i, j, k], M_ij)

    # ────────────────────────────────
    #  RÉSOLUTION
    # ────────────────────────────────
    start = time.time()
    status_code = solver.Solve()
    solve_time  = time.time() - start

    status_map = {
        pywraplp.Solver.OPTIMAL:    'OPTIMAL',
        pywraplp.Solver.FEASIBLE:   'FEASIBLE',
        pywraplp.Solver.INFEASIBLE: 'INFEASIBLE',
        pywraplp.Solver.NOT_SOLVED: 'TIMEOUT',
    }
    status = status_map.get(status_code, 'UNKNOWN')

    if status not in ('OPTIMAL', 'FEASIBLE'):
        return {
            'status':       status,
            'objective':    None,
            'routes':       [],
            'solve_time':   solve_time,
            'num_vehicles': 0
        }

    # ── Extraction des routes ──
    routes = []
    for k in K:
        # Cherche si ce véhicule est utilisé
        if not any(x[0, j, k].solution_value() > 0.5
                   for j in N if j != 0):
            continue
        route = [0]
        current = 0
        visited = set()
        while True:
            next_node = None
            for j in N:
                if j != current and j not in visited:
                    if x[current, j, k].solution_value() > 0.5:
                        next_node = j
                        break
            if next_node is None or next_node == 0:
                route.append(0)
                break
            route.append(next_node)
            visited.add(next_node)
            current = next_node
        routes.append(route)

    return {
        'status':       status,
        'objective':    solver.Objective().Value(),
        'routes':       routes,
        'solve_time':   solve_time,
        'num_vehicles': len(routes)
    }


# ─────────────────────────────────────────────
#  QUESTION 6 : ANALYSE DE LA LIMITE
# ─────────────────────────────────────────────

def run_scalability_analysis(sizes=None, time_limit=120, seed=42,
                               with_time_windows=True, base_instance=None):
    """
    Fait tourner le solver ILP sur des instances de taille croissante
    et mesure le temps de résolution.

    Paramètres
    ----------
    sizes      : liste du nombre de clients à tester
    time_limit : limite de temps par instance (secondes)
    seed       : graine aléatoire

    Retourne un DataFrame-like (liste de dicts) avec les résultats.
    """
    if sizes is None:
        sizes = [3, 5, 7, 10, 12, 15, 18, 20, 25]

    results = []
    print(f"\n{'='*60}")
    print(f"  ANALYSE DE SCALABILITÉ DU SOLVEUR ILP (TW={with_time_windows})")
    print(f"{'='*60}")
    print(f"{'Clients':>10} | {'Statut':>12} | {'Temps (s)':>10} | {'Objectif':>12}")
    print(f"{'-'*55}")

    for n in sizes:
        if base_instance:
            instance = scale_instance(base_instance, n_clients=n, seed=seed)
        else:
            instance = generate_random_instance(
                n_clients=n, capacity=200, seed=seed
            )
        result = solve_vrptw_ilp(
            instance,
            time_limit_seconds=time_limit,
            with_time_windows=with_time_windows
        )

        results.append({
            'n_clients':   n,
            'status':      result['status'],
            'solve_time':  result['solve_time'],
            'objective':   result['objective'],
            'n_vehicles':  result['num_vehicles']
        })

        obj_str = f"{result['objective']:.2f}" if result['objective'] else "N/A"
        print(f"{n:>10} | {result['status']:>12} | "
              f"{result['solve_time']:>10.2f} | {obj_str:>12}")

    return results


def plot_scalability(results, output_path="scalability.png"):
    """
    Trace les courbes de scalabilité :
    - Temps de résolution vs nombre de clients
    - Statut de chaque résolution (couleur)
    """
    ns      = [r['n_clients']  for r in results]
    times   = [r['solve_time'] for r in results]
    statuts = [r['status']     for r in results]

    colors = {
        'OPTIMAL':    'green',
        'FEASIBLE':   'orange',
        'INFEASIBLE': 'red',
        'TIMEOUT':    'gray',
        'UNKNOWN':    'black'
    }

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    # ── Graphe 1 : temps de résolution ──
    bar_colors = [colors.get(s, 'blue') for s in statuts]
    ax1.bar(ns, times, color=bar_colors, edgecolor='black', alpha=0.8)
    ax1.set_xlabel("Nombre de clients", fontsize=12)
    ax1.set_ylabel("Temps de résolution (s)", fontsize=12)
    ax1.set_title("Temps de résolution ILP vs Taille de l'instance", fontsize=13)
    ax1.set_xticks(ns)

    # Légende statuts
    from matplotlib.patches import Patch
    legend = [Patch(color=c, label=s) for s, c in colors.items()
              if s in statuts]
    ax1.legend(handles=legend, loc='upper left')

    # ── Graphe 2 : courbe log ──
    optimal_ns    = [r['n_clients']  for r in results if r['status'] == 'OPTIMAL']
    optimal_times = [r['solve_time'] for r in results if r['status'] == 'OPTIMAL']
    ax2.plot(optimal_ns, optimal_times, 'go-', label='OPTIMAL', linewidth=2)

    feasible_ns    = [r['n_clients']  for r in results if r['status'] == 'FEASIBLE']
    feasible_times = [r['solve_time'] for r in results if r['status'] == 'FEASIBLE']
    if feasible_ns:
        ax2.plot(feasible_ns, feasible_times, 'o-',
                 color='orange', label='FEASIBLE', linewidth=2)

    timeout_ns = [r['n_clients'] for r in results if r['status'] == 'TIMEOUT']
    for nt in timeout_ns:
        ax2.axvline(x=nt, color='red', linestyle='--', alpha=0.5)

    ax2.set_xlabel("Nombre de clients", fontsize=12)
    ax2.set_ylabel("Temps (s) - échelle log", fontsize=12)
    ax2.set_title("Croissance exponentielle du temps de résolution", fontsize=13)
    ax2.set_yscale('log')
    ax2.legend()
    ax2.grid(True, which='both', alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"\nGraphique sauvegardé : {output_path}")


# ─────────────────────────────────────────────
#  VISUALISATION DES ROUTES
# ─────────────────────────────────────────────

def plot_routes(instance, result, output_path="routes.png"):
    """Visualise les routes de la solution."""
    if not result['routes']:
        print("Aucune route à afficher.")
        return

    fig, ax = plt.subplots(figsize=(10, 8))
    colors = plt.cm.tab10.colors

    # Clients
    for c in instance.clients[1:]:
        ax.scatter(c.x, c.y, color='steelblue', s=80, zorder=5)
        ax.annotate(f"{c.id}", (c.x, c.y),
                    textcoords="offset points", xytext=(5, 5), fontsize=8)

    # Dépôt
    depot = instance.clients[0]
    ax.scatter(depot.x, depot.y, color='red', s=200,
               marker='*', zorder=6, label='Dépôt')

    # Routes
    for idx, route in enumerate(result['routes']):
        color = colors[idx % len(colors)]
        coords = [(instance.clients[n].x, instance.clients[n].y) for n in route]
        xs, ys = zip(*coords)
        ax.plot(xs, ys, '-o', color=color, linewidth=1.5,
                alpha=0.7, label=f'Véhicule {idx+1}')

    ax.set_title(
        f"Routes VRPTW | Dist totale: {result['objective']:.2f} | "
        f"Véhicules: {result['num_vehicles']} | Statut: {result['status']}",
        fontsize=11
    )
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Routes sauvegardées : {output_path}")


# ─────────────────────────────────────────────
#  POINT D'ENTRÉE
# ─────────────────────────────────────────────

if __name__ == "__main__":

    print("\n" + "="*60)
    print("  VRPTW - Résolution ILP avec OR-Tools")
    print("="*60)

    # ── [Nouveau] Résolution de l'instance réelle data101.vrp ──
    import os
    base_path = "data/raw/data101.vrp"
    if os.path.exists(base_path):
        print(f"\n[2] Résolution de l'instance réelle : {base_path} (100 clients)")
        # On utilise le parseur officiel du projet
        core_inst = VRPParser.parse(base_path)
        instance_101 = VRPTWInstance(core_inst.clients, core_inst.capacity)
        
        # Résolution (temps limite de 10 minutes)
        result_101 = solve_vrptw_ilp(instance_101, time_limit_seconds=600)
        
        print(f"\n    Résultat pour data101 :")
        print(f"    Statut    : {result_101['status']}")
        print(f"    Objectif  : {result_101['objective']:.2f}" if result_101['objective'] else "    Objectif  : N/A")
        print(f"    Véhicules : {result_101['num_vehicles']}")
        print(f"    Temps     : {result_101['solve_time']:.3f}s")
        
        # Visualisation
        if result_101['routes']:
            plot_routes(instance_101, result_101, "routes_data101.png")
            print(f"    Image sauvegardée : routes_data101.png")
    else:
        print(f"\n[!] Fichier {base_path} introuvable.")

    # ── Test sur une petite instance aléatoire ──
    print("\n[1] Test sur une instance aléatoire (8 clients)")
    instance = generate_random_instance(n_clients=8, capacity=200, seed=42)
    print(f"    Capacité : {instance.capacity}")
    print(f"    Clients  : {instance.n}")

    result = solve_vrptw_ilp(instance, time_limit_seconds=60,
                             with_time_windows=True)
    print(f"\n    Statut    : {result['status']}")
    print(f"    Objectif  : {result['objective']:.2f}" if result['objective'] else "    Objectif  : N/A")
    print(f"    Véhicules : {result['num_vehicles']}")
    print(f"    Temps     : {result['solve_time']:.3f}s")
    print(f"    Routes    :")
    for i, r in enumerate(result['routes']):
        print(f"      Véhicule {i+1} : {' -> '.join(map(str, r))}")

    # Visualisation des routes
    plot_routes(instance, result, "routes_test.png")

    # ── Question 6 : analyse de scalabilité ──
    print("\n[2] Analyse de scalabilité (Question 6)")
    # On charge une instance réelle pour le Bonus
    import os
    base_path = "data/raw/data101.vrp"
    if os.path.exists(base_path):
        # On utilise le parseur officiel du projet qui gère le format .vrp
        core_inst = VRPParser.parse(base_path)
        base_instance = VRPTWInstance(core_inst.clients, core_inst.capacity)
    else:
        base_instance = None
    
    sizes  = [100] 
    results = run_scalability_analysis(
        sizes=sizes,
        time_limit=600, # 10 minutes max pour voir la limite
        with_time_windows=True,
        base_instance=base_instance
    )

    plot_scalability(results, "scalability.png")

    # ── Résumé ──
    print("\n[3] Résumé")
    for r in results:
        flag = "[!]" if r['status'] in ('TIMEOUT', 'INFEASIBLE') else "[OK]"
        print(f"  {flag}  {r['n_clients']:>3} clients -> "
              f"{r['status']:>12} en {r['solve_time']:.2f}s")