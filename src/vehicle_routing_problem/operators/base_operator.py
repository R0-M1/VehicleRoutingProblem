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

    @classmethod
    @abstractmethod
    def generate_neighbors(cls, instance: Instance, solution: Solution) -> list[BaseOperator]:
        """
        Génère tous les opérateurs de voisinage possibles
        """
        ...

    @abstractmethod
    def affected_routes(self) -> list[int]:
        """Retourne les indices des routes modifiées par cet opérateur."""
        ...

    @classmethod
    @abstractmethod
    def sample_random_neighbor(cls, instance: Instance, solution: Solution) -> BaseOperator | None:
        """
        Génère un seul opérateur de voisinage aléatoire valide, ou None si impossible.
        """
        ...

    def get_signature(self) -> tuple:
        """
        Retourne une signature unique et hashable de cet opérateur, 
        basée sur son type et ses attributs.
        Utile pour la liste tabou en O(1).
        """
        # On extrait les valeurs de tous les attributs, sauf l'instance (_inst) qui n'est pas hashable
        # sorted() assure que l'ordre des attributs est toujours le même
        attributes = tuple(v for k, v in sorted(self.__dict__.items()) if k != '_inst')
        return (self.__class__.__name__, attributes)

    @classmethod
    def get_operators(cls, *names):
        return BaseOperator.__subclasses__()