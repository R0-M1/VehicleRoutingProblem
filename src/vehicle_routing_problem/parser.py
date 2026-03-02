import math
import numpy as np
from model import Node


def load_instance(filepath: str):
    """
    Parse un fichier personnalisé Polytech (.vrp)
    Retourne la liste des Noeuds, la capacité max, et la matrice de distances.
    """
    nodes = []
    capacity = 0

    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    current_section = "META"
    node_id = 0  # L'ID 0 sera le dépôt, les suivants seront les clients

    for line in lines:
        line = line.strip()
        if not line or line.startswith("COMMENT") or line.startswith("TYPE") or line.startswith("COORDINATES"):
            continue

        # 1. Lecture de la Capacité
        if line.startswith("MAX_QUANTITY:"):
            capacity = int(line.split(":")[1].strip())

        # 2. Détection des sections
        elif line.startswith("DATA_DEPOTS"):
            current_section = "DEPOTS"
        elif line.startswith("DATA_CLIENTS"):
            current_section = "CLIENTS"

        # 3. Lecture du Dépôt
        elif current_section == "DEPOTS" and line.startswith("d"):
            parts = line.split()
            # Format: idName(0) x(1) y(2) readyTime(3) dueTime(4)
            nodes.append(Node(
                id=node_id,
                x=float(parts[1]),
                y=float(parts[2]),
                demand=0,  # Le dépôt n'a pas de demande
                tw_start=int(parts[3]),
                tw_end=int(parts[4]),
                service_time=0  # Le dépôt n'a pas de temps de service
            ))
            node_id += 1

        # 4. Lecture des Clients
        elif current_section == "CLIENTS" and line.startswith("c"):
            parts = line.split()
            # Format: idName(0) x(1) y(2) readyTime(3) dueTime(4) demand(5) service(6)
            nodes.append(Node(
                id=node_id,
                x=float(parts[1]),
                y=float(parts[2]),
                tw_start=int(parts[3]),
                tw_end=int(parts[4]),
                demand=int(parts[5]),
                service_time=int(parts[6])
            ))
            node_id += 1

    # 5. Calcul de la Matrice de Distances Euclidiennes (Théorème de Pythagore)
    n = len(nodes)
    dist_matrix = np.zeros((n, n))

    for i in range(n):
        for j in range(n):
            if i != j:
                dx = nodes[i].x - nodes[j].x
                dy = nodes[i].y - nodes[j].y
                # Arrondi à 2 décimales (souvent recommandé dans les TP pour éviter les erreurs de flottants)
                dist_matrix[i][j] = round(math.hypot(dx, dy), 2)

    return nodes, capacity, dist_matrix
