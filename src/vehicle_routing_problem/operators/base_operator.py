from __future__ import annotations
from abc import ABC, abstractmethod
from vehicle_routing_problem.core.solution import Solution
from vehicle_routing_problem.core.instance import Instance

class BaseOperator(ABC):
    """
    Classe abstraite commune à tous les opérateurs de voisinage.
    Chaque sous-classe reçoit ses paramètres spécifiques dans son constructeur
    et implémente apply().
    """

    def __init__(self, instance: Instance):
        self._inst = instance

    @abstractmethod
    def apply(self, solution: Solution) -> Solution:
        """
        Applique l'opérateur sur la solution donnée avec les paramètres
        du constructeur. Retourne la nouvelle solution résultante.
        """
        ...

    @abstractmethod
    def generate_neighbors(self, solution: Solution) -> list[BaseOperator]:
        """
        Génère tous les opérateurs de voisinage possibles pour la solution donnée.
        """
        ...